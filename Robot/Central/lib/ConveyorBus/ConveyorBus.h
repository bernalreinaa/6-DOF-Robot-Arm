// ConveyorBus — comunicación ESP-NOW del Central con la cinta transportadora.
#pragma once
#include <Arduino.h>
#include "ConveyorProtocol.h"

class ConveyorBus {
public:
    void begin();  // registra la cinta como peer ESP-NOW

    void setVelocityPercent(float pct) { velocityPercent_ = constrain(pct, 0.0f, 100.0f); }
    void setRun(bool run) { run_ = run; }
    void setDetectionThresholdCm(float cm) { detectionThresholdCm_ = cm; }

    float velocityPercent() const { return velocityPercent_; }
    bool  run() const { return run_; }

    // Reenvía el struct completo con los últimos valores conocidos. Se
    // llama cada vez que cambia CUALQUIERA de los tres campos.
    void sendCommand();

    // Actualiza el último estado conocido (llamar desde el callback ESP-NOW
    // del Central al recibir un ConveyorStatus).
    void onStatusReceived(const ConveyorStatus& status);

    bool  objectDetected() const { return objectDetected_; }
    float lastDistanceCm() const { return lastDistanceCm_; }

private:
    float velocityPercent_ = 50.0f;
    bool  run_ = false;
    float detectionThresholdCm_ = 4.0f;  // mismo valor por defecto que trae la cinta de fábrica

    bool  objectDetected_ = false;
    float lastDistanceCm_ = -1.0f;
};
