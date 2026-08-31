// MagneticEncoder — envoltorio del sensor magnético AS5600 con conteo de
// vueltas, para obtener un ángulo ABSOLUTO acumulado (puede superar 360°)
// en vez del ángulo módulo-360 crudo que da el sensor. Agnóstico de la
// reductora: la conversión a grados de salida la hace JointMotionController.
#pragma once
#include <Arduino.h>

class MagneticEncoder {
public:
    // sdaPin/sclPin: bus I2C compartido con la pantalla OLED.
    void begin(int sdaPin, int sclPin, uint32_t i2cClockHz = 100000);

    bool isConnected();

    // El AS5600 se monta invertido respecto al sentido de giro "positivo"
    // definido para este proyecto; se corrige aquí una sola vez en begin().
    void setDirectionCounterClockwise();

    // Lee el sensor, actualiza el conteo de vueltas y devuelve el ángulo
    // RAW acumulado (turns*360 + ángulo_módulo_360), en el espacio físico
    // del propio sensor — sin dividir por ninguna reductora. Llamar
    // periódicamente (p.ej. cada 10 ms) desde la tarea de lectura.
    double pollAccumulatedDegrees();

private:
    double lastRawModDeg_ = 0.0;
    long   turns_ = 0;
    bool   firstSample_ = true;
};
