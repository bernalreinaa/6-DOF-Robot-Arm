// ConveyorProtocol — estructuras del protocolo ESP-NOW entre el Central y
// la cinta transportadora (Cinta_Transportadora_ESP32C3).
//
// IMPORTANTE: debe ser IDÉNTICO (mismo orden y tipos) en ambos proyectos —
// el receptor distingue el tipo de paquete por su tamaño (sizeof).
#pragma once
#include <Arduino.h>

// Central -> Cinta: arranque/paro, velocidad y distancia umbral de detección.
struct ConveyorCommand {
    float   velocityPercent;   // % velocidad objetivo (0-100)
    boolean run;               // true = arrancar, false = parar
    float   detectionThresholdCm;  // distancia umbral de detección (cm); <=0 = "sin cambio"
};

// Cinta -> Central: objeto detectado y última distancia medida.
// Tamaño (12B) elegido a propósito distinto de AngleMessage (8B) para que
// el receptor no los confunda (discriminación por sizeof).
struct ConveyorStatus {
    int     marker;   // valor fijo distintivo, no usado como dato (ver firmware de la cinta)
    boolean objectDetected;
    float   distanceCm;
};
