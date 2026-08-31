// EspNowProtocol — estructuras del protocolo ESP-NOW compartido entre el
// Central (ESP32-S3), el Mando (ESP32-S3) y las 6 articulaciones (ESP32-C3).
//
// IMPORTANTE: este fichero debe ser IDÉNTICO (mismo orden y tipos de campos)
// en los 8 proyectos que lo usan (Central, Mando, Articulacion_1..6),
// porque el receptor distingue el tipo de paquete por su tamaño (sizeof),
// no por un campo de tipo explícito. Si un solo proyecto se queda
// desactualizado, sus paquetes dejan de coincidir en tamaño y se descartan
// en silencio. Cualquier cambio aquí debe replicarse en los 8 a la vez.
#pragma once
#include <Arduino.h>

// Central -> Articulación: orden de recarga de parámetros por Serial ("reload[i]").
struct ReloadCommand {
    boolean reload;
};

// Articulación -> Central / Central -> Mando: ángulo de salida actual.
struct AngleMessage {
    int   id;         // 1..6
    float angleDeg;   // ángulo de salida (grados)
};

// Articulación -> Central (al arrancar) / Central -> Articulación (al
// recibir "init[...]" por Serial): parámetros de ajuste (PID, velocidades,
// zonas, límites de zona prohibida). Un único layout se reutiliza en ambos
// sentidos.
struct TuningParams {
    int    id;                 // 1..6
    float  kp;
    float  ki;
    float  kd;
    float  maxVelDegS;
    float  cruiseVelDegS;
    float  approachVelDegS;
    float  minVelDegS;
    float  tolDeg;
    float  slowZoneDeg;
    float  approachZoneDeg;
    double limitInfDeg;
    double limitSupDeg;
};

// Central -> Articulación / Mando -> Central: setpoint + reset + enable +
// porcentaje de velocidad para el movimiento en curso.
//
// NOTA sobre "motorDisabled": por compatibilidad con el hardware existente
// (TB6600, pin ENA activo en HIGH = motor deshabilitado) el campo conserva
// la semántica invertida original: true = deshabilitar motor, false =
// habilitarlo. Se documenta aquí para que quede explícito en todo el código
// que lo consume.
struct SetpointCommand {
    int     id;                // 1..6 (0 = comando de emergencia global, ver Central/Mando)
    float   setpointDeg;
    boolean reset;
    boolean motorDisabled;
    float   velocityPercent;   // 5-100 %; 100 = velocidad normal
};
