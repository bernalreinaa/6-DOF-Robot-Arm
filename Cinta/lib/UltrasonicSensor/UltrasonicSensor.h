// UltrasonicSensor — envoltorio del sensor HC-SR04 con captura del pulso de
// ECHO por INTERRUPCIÓN (no pulseIn()) y filtro de mediana sobre las
// últimas N lecturas.
//
// Motivo de la ISR en vez de pulseIn(): pulseIn() sondea el pin desde la
// propia tarea, y el driver de WiFi/ESP-NOW (tareas internas de más
// prioridad) puede apartar la tarea de la CPU varios cientos de
// microsegundos justo durante la medida, corrompiendo el tiempo de pulso.
// La ISR captura los instantes de subida/bajada con micros() con mucha más
// puntualidad, aunque el WiFi esté activo.
//
// Motivo del filtro de mediana: el motor+driver TB6600 conmutando cerca del
// sensor mete ruido eléctrico que de vez en cuando produce una lectura
// suelta y disparatada (eco espurio); la mediana de las últimas N lecturas
// descarta ese tipo de valor atípico aislado.
#pragma once
#include <Arduino.h>

class UltrasonicSensor {
public:
    // medianSamples debe ser >= 1; se reserva internamente (máx. razonable,
    // ver ConveyorConfig::medianSamples).
    void begin(int trigPin, int echoPin, unsigned long timeoutUs, int medianSamples);

    // Dispara una medición y espera (con timeout) el pulso de ECHO. Devuelve
    // la mediana filtrada en cm, o -1.0 si no hay lectura válida reciente.
    // Llamar periódicamente desde una tarea (no es bloqueante más allá del
    // timeout configurado).
    float measureCm();

private:
    int trigPin_ = -1;
    int echoPin_ = -1;
    unsigned long timeoutUs_ = 20000UL;

    static constexpr int kMaxMedianSamples = 9;
    float samples_[kMaxMedianSamples];
    int medianSamples_ = 3;
    int sampleIndex_ = 0;
};
