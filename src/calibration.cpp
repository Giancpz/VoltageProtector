#include <Arduino.h>
#include "calibration.h"

int calibrate()
{
  int max = 0;
  int min = 0;
  for (int i = 0; i < 20; i++)
  {
    long start = millis();
    int temp_max = 0;
    int temp_min = 10000000;
    for (;;)
    {
      int samples = 0;
      for (int i = 0; i < 20; i++)
      {
        samples += analogRead(2);
        delay(1);
      }

      int prom = samples / 20;

      if (prom > temp_max)
      {
        temp_max = prom;
      }

      if (prom < temp_min)
      {
        temp_min = prom;
      }

      if ((millis() - start) > 100)
      {
        max += temp_max;
        min += temp_min;
        break;
      }
    }
  }

  digitalWrite(35, HIGH);
  delay(500);
  digitalWrite(35, LOW);
  return (max / 20.0) + (min / 20.0) / 2;
}