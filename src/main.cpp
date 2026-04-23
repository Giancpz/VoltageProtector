#include <Arduino.h>
#include <LiquidCrystal.h>
#include <driver/adc.h>
#include <string.h>
#include "esp_task_wdt.h"
#include "calibration.h"
#include "log.h"

const uint8_t SENSOR = 2;
const uint8_t RELAY = 6;
const uint8_t LCD_BACKLIGHT = 5;
const uint8_t TOUCH = 7;

enum State
{
  NORMAL,
  LOW_VOLTAGE,
  OVER_VOLTAGE,
  DISCONNECTED,
  TESTING,
  WAITING
};

SimpleLogger logger(15);
LiquidCrystal lcd(34, 33, 47, 48, 26, 21);

TaskHandle_t analyzer_handle, comunication_handle;
const uint8_t SAMPLES = 16;
const uint16_t timingDuration = 1041;

volatile bool writing = false;

volatile bool filter = false;
volatile State state = TESTING;

volatile int16_t voltage[16];
volatile int16_t min_voltage[16];
volatile int16_t max_voltage[16];
volatile int32_t timee[16];
int16_t temp_voltage[SAMPLES];
int16_t temp_last_voltage[SAMPLES];
int16_t temp_min_voltage[SAMPLES];
int16_t temp_max_voltage[SAMPLES];

volatile int16_t frecuency = 0;
volatile float cycle_time = 0;
volatile float v_rms = 0;

float mid_flotante = 500.0;
const float alpha = 0.001;
float mid = 500;
SemaphoreHandle_t xMutex;

uint16_t max_test = 9600;
uint16_t correct_test = 0;
uint32_t last_disconnect = 0;

volatile bool lcd_on = true;

struct __attribute__((packed)) ACData
{
  int16_t voltage[SAMPLES];
  int16_t min_voltage[SAMPLES];
  int16_t max_voltage[SAMPLES];
  int32_t time[SAMPLES];
  int16_t mid;
  int16_t frecuency;
  float cycle_time;
};

ACData acdata;

int32_t low_protection[16] = {20, 54, 100, 131, 141, 131, 100, 54, 0, -54, -100, -131, -141, -131, -100, -54};

int32_t over_protection[16] = {30, 76, 140, 183, 198, 183, 140, 76, 0, -76, -140, -183, -198, -183, -140, -76};

void Analyzer(void *pvParameters);
void Comunication(void *pvParameters);
void Alert(void *pvParameters);
void LCD(void *pvParameters);
void AC_Error(State cause);
void SendMessage(String text);

void setup()
{
  Serial.begin(115200);
  pinMode(SENSOR, INPUT);
  pinMode(TOUCH, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(LCD_BACKLIGHT, OUTPUT);

  digitalWrite(SENSOR, LOW);
  digitalWrite(RELAY, LOW);
  digitalWrite(LCD_BACKLIGHT, HIGH);
  analogReadResolution(10);

  pinMode(34, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(21, OUTPUT);

  digitalWrite(34, LOW);
  digitalWrite(33, LOW);
  digitalWrite(47, LOW);
  digitalWrite(48, LOW);
  digitalWrite(26, LOW);
  digitalWrite(21, LOW);
  delay(100);

  pinMode(35, OUTPUT);

  logger.add("Loading...");
  lcd.clear();
  delay(100);
  lcd.begin(16, 2);
  lcd.print("AC Analyzer");

  // memset(voltage, 0, sizeof(voltage));
  // memset(min_voltage, 0, sizeof(min_voltage));
  // memset(max_voltage, 0, sizeof(max_voltage));

  for (int i = 0; i < 16; i++)
  {
    temp_voltage[i] = 0;
    temp_min_voltage[i] = 0;
    temp_max_voltage[i] = 0;
  }

  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = 10000,
      .idle_core_mask = 0,
      .trigger_panic = true};

  esp_task_wdt_reconfigure(&twdt_config);

  mid = calibrate();
  logger.add("Calibrated: ");

  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(Analyzer, "Analyzer", 8196, NULL, 1, &analyzer_handle, 0);
  logger.add("Created task: Analyzer");
  delay(200);

  xTaskCreatePinnedToCore(LCD, "LCD", 4096, NULL, 2, NULL, 1);
  logger.add("Created task: LCD");
  xTaskCreatePinnedToCore(Comunication, "Comunication", 4096, NULL, 4, &comunication_handle, 1);
  logger.add("Created task: Comunication");
  //xTaskCreatePinnedToCore(Alert, "Alert", 2048, NULL, 1, NULL, 1);
  //logger.add("Created task: Alert");
  logger.dump();
}

void loop()
{
  delayMicroseconds(100);
}

void Analyzer(void *pvParameters)
{
  // esp_task_wdt_add(NULL);
  bool newCycle0 = false;
  int count = 0;

  unsigned long tiempoAnterior = 0;

  long temp_time[SAMPLES];

  long cycle_time_1 = 0;

  mid = (int)mid_flotante;

  logger.add("Loaded analyzer");

  int error_count = 0;

  for (;;)
  {
    int suma = analogRead(SENSOR);
    suma += analogRead(SENSOR);
    int prom = suma / 2;
    mid_flotante = (alpha * prom) + ((1.0 - alpha) * mid_flotante);

    unsigned long tiempoActual = micros();
    if (prom > mid && newCycle0)
    {
      // temp_voltage[count] = value;
      // temp_time[count] = micros() - cycle_time_1;

      if (xSemaphoreTake(xMutex, 0) == pdTRUE)
      {
        memcpy((void *)voltage, temp_voltage, sizeof(temp_voltage));
        memcpy((void *)min_voltage, temp_min_voltage, sizeof(temp_min_voltage));
        memcpy((void *)max_voltage, temp_max_voltage, sizeof(temp_max_voltage));
        memcpy((void *)timee, temp_time, sizeof(temp_time));
        memcpy((void *)temp_last_voltage, temp_voltage, sizeof(temp_voltage));
        xSemaphoreGive(xMutex);
      }
      
      newCycle0 = false;
      mid = (int)mid_flotante;

      cycle_time = (micros() - cycle_time_1) / 1000.0;

      cycle_time_1 = micros();
      tiempoAnterior = tiempoActual;
      error_count = 0;

      float value = (prom - mid) * 1.7;
      temp_voltage[0] = value;
      temp_time[0] = micros() - cycle_time_1;

      count = 1;
    }

    if (prom < (mid - 30) && !newCycle0)
    {
      newCycle0 = true;
    }

    long t = micros() - cycle_time_1;

    if (t > 17000 && state != DISCONNECTED)
    {
      AC_Error(DISCONNECTED);
    }
    else
    {
      if(state == DISCONNECTED)
      {
        AC_Error(WAITING);
      }
    }

    if (count < 16)
    {
      if (tiempoActual - tiempoAnterior >= timingDuration)
      {
        tiempoAnterior += timingDuration;

        long m = millis();

        bool high_side = count <= 7;
        bool low_side = count >= 9;
        float value;

        float value0 = (prom - mid) * 1.7;

        const float alpha = 0.1;

        if (false)
        {
          value = (alpha * value0) + ((1.0 - alpha) * temp_last_voltage[count]);
        }
        else
        {
          value = value0;
        }

        bool blow_protection = false;
        bool bover_protection = false;

        blow_protection = (high_side && value < (low_protection[count] - 25)) || (low_side && value > (low_protection[count] + 25));

        bover_protection = (high_side && value > (over_protection[count] + 10)) || (low_side && value < (over_protection[count] - 10));

        if (blow_protection)
        {
          error_count++;
          blow_protection = false;
          
          if(error_count > 1)
          {
            AC_Error(LOW_VOLTAGE);
          }
        } 
        else if(bover_protection)
        {
          error_count++;
          bover_protection = false;

          if(error_count > 1) 
          {
            AC_Error(OVER_VOLTAGE);
          }
        }
        else
        {
          if(state == TESTING)
          {
            if (correct_test < max_test)
            {
              correct_test++;
            }
          }
        }

        if (high_side && state == NORMAL)
        {
          if (value > temp_max_voltage[count] || temp_max_voltage[count] == 0)
          {
            temp_max_voltage[count] = value;
          }

          if (value < temp_min_voltage[count] || temp_min_voltage[count] == 0)
          {
            temp_min_voltage[count] = value;
          }
        }

        if (low_side && state == NORMAL)
        {
          if (value < temp_max_voltage[count] || temp_max_voltage[count] == 0)
          {
            temp_max_voltage[count] = value;
          }

          if (value > temp_min_voltage[count] || temp_min_voltage[count] == 0)
          {
            temp_min_voltage[count] = value;
          }
        }

        temp_voltage[count] = value;
        temp_time[count] = micros() - cycle_time_1;

        high_side = false;
        low_side = false;
        count++;
      }
    }

    if(state == NORMAL)
    {
      digitalWrite(RELAY, HIGH);
    }
    else
    {
      digitalWrite(RELAY, LOW);
    }

    if (state != NORMAL && state != DISCONNECTED)
    {
      if(state != TESTING)
      {
        if (millis() - last_disconnect > 10000)
        {
          state = TESTING;
        }
      }

      if(state == TESTING)
      {
        if(correct_test >= max_test)
        {
          state = NORMAL;
        }
      }
    }
    else
    {
      correct_test = 0;
    }

    delayMicroseconds(6);
    // esp_task_wdt_reset();
  }
  logger.add("Error ocurred");
}

void AC_Error(State cause)
{
  state = cause;
  last_disconnect = millis();
  correct_test = 0;
}

void Comunication(void *pvParameters)
{
  uint8_t header[] = {0xAA, 0xBB, 0xCC};
  // USBSerial.begin(115200);
  for (;;)
  {
    if (Serial.available() > 0)
    {
      uint8_t r = Serial.read();

      if (r == 0xA1)
      {
        logger.dump();
      }

      if (r == 0xA2)
      {
          TickType_t tiempoMaximo = pdMS_TO_TICKS(500);

          if (xSemaphoreTake(xMutex, tiempoMaximo) == pdTRUE) 
          {
            memcpy(acdata.voltage, (void *)voltage, sizeof(voltage));
            memcpy(acdata.min_voltage, (void *)min_voltage, sizeof(min_voltage));
            memcpy(acdata.max_voltage, (void *)max_voltage, sizeof(max_voltage));
            memcpy(acdata.time, (void *)timee, sizeof(timee));
            acdata.mid = mid;
            acdata.cycle_time = cycle_time;
            acdata.frecuency = frecuency;
          xSemaphoreGive(xMutex);
          
          Serial.write(header, 3);
          Serial.write((uint8_t *)&acdata, sizeof(acdata));
        }
      }

      if (r == 0xA3)
      {
        filter = true;
      }
      if (r == 0xA4)
      {
        filter = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void SendMessage(String text)
{
  uint8_t header[] = {0xAA, 0xBB, 0xDD};
  Serial.write(header, 3);
  Serial.println(text);
}

void Alert(void *pvParameters)
{
  for (;;)
  {
    vTaskDelay(500);
  }
}


long antgt = 0;
long antgtgrg = 0;

uint8_t testing_count = 0;

void LCD(void *pvParameters)
{
  lcd.clear();
  for (;;)
  {
    long sumaCuadrados = 0;
    for (int i = 0; i < SAMPLES; i++)
    {
      sumaCuadrados += (voltage[i] * voltage[i]);
    }
    v_rms = sqrt(sumaCuadrados / (float)SAMPLES);

    if(touchRead(TOUCH) > 170000 || state != NORMAL)
    {
      antgt = millis();
      lcd_on = true;
      digitalWrite(LCD_BACKLIGHT, HIGH);
    }

    if (millis() - antgt > 30000)
    {
      lcd_on = false;
      digitalWrite(LCD_BACKLIGHT, LOW);
    }

    if (millis() - antgtgrg > 500)
    {
      antgtgrg = millis();
      int frecuency = 1000.0 / (cycle_time - 0.1);

      lcd.setCursor(0, 0);
      switch (state)
      {
      case NORMAL:
        lcd.print("AC: Normal        ");
        break;
      case LOW_VOLTAGE:
        lcd.print("AC: Low voltage   ");
        break;
      case OVER_VOLTAGE:
        lcd.print("AC: OVER VOLTAGE! ");
        break;
      case DISCONNECTED:
        lcd.print("AC: Disconnect    ");
        break;
      case TESTING:

        if(testing_count == 1){
          lcd.print("AC: TESTING.    ");
        }
        if(testing_count == 2){
          lcd.print("AC: TESTING..   ");
        }
        if(testing_count == 3)
        {
          lcd.print("AC: TESTING...    ");
          testing_count = 0;
        }
        testing_count++;
        break;

      default:
        break;
      }

      lcd.setCursor(3, 1);
      lcd.print((int)v_rms);
      lcd.print("v");
      lcd.setCursor(8, 1);
      lcd.print(frecuency);
      lcd.print("hz    ");
    }
    vTaskDelay(100);
  }
}
