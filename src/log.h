#ifndef SIMPLE_LOGGER_H
#define SIMPLE_LOGGER_H

#include <Arduino.h>

struct LogEntry
{
    unsigned long timestamp;
    char message[50];
};

class SimpleLogger
{
private:
    int _size;
    int _head;
    LogEntry *_buffer;

public:
    // Constructor: tú decides de qué tamaño será el log al crear la instancia
    SimpleLogger(int size)
    {
        _size = size;
        _head = 0;
        _buffer = new LogEntry[_size];
        // Inicializamos los tiempos en 0
        for (int i = 0; i < _size; i++)
            _buffer[i].timestamp = 0;
    }

    // Destructor: para liberar memoria si la instancia se destruye
    ~SimpleLogger()
    {
        delete[] _buffer;
    }

    void add(const char *msg)
    {
        _buffer[_head].timestamp = millis();
        strncpy(_buffer[_head].message, msg, 49);
        _buffer[_head].message[49] = '\0'; // Aseguramos el fin de cadena

        _head = (_head + 1) % _size; // Movimiento circular
    }

    void dump()
    {
        uint8_t header[] = {0xAA, 0xBB, 0xDD};
        for (int i = 0; i < _size; i++)
        {
            int index = (_head + i) % _size;
            if (_buffer[index].timestamp > 0)
            {
                uint8_t header[] = {0xAA, 0xBB, 0xEE};
                Serial.write(header, 3);
                Serial.println(_buffer[index].message);
            }
        }
    }
};

#endif