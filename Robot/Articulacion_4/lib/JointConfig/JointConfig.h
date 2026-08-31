// JointConfig — ÚNICO fichero que cambia entre las 6 articulaciones.
// Todo el resto del firmware (lib/ + src/main.cpp) es idéntico; solo estos
// valores (mecánica, ganancias PID, perfil de velocidad y zona prohibida)
// distinguen una articulación de otra.
#pragma once
#include <Arduino.h>

namespace jointConfig {

constexpr int id = 4;

// ── Pines (iguales en las 6 placas ESP32-C3) ────────────────────────────
constexpr int i2cSdaPin = 5;
constexpr int i2cSclPin = 6;
constexpr int pinDir    = 4;
constexpr int pinPul    = 3;
constexpr int pinEnable = 8;

// ── Mecánica: reductora, microstepping y sentido ────────────────────────
constexpr float ratio = 3.927f;             // Reductora articulación 4
constexpr int   driverMicrosteps = 8;
constexpr int   motorStepsPerRev = 200;
constexpr bool  invertDir = false;

// ── Encoder AS5600: mide directamente el eje del motor (sin gear propio) ──
constexpr bool  encoderOnOutputSide = false;
constexpr float encoderGearRatio = 1.0f;    // sin uso (encoderOnOutputSide == false)

// ── Compensación de backlash (holgura de reductora) ─────────────────────
constexpr float backlashOutDeg = 0.0f;

// ── Zona prohibida (grados de SALIDA, 0-360) ────────────────────────────
// Zona prohibida entre 91° y 269° -> rango útil (269°,360°) U (0°,91°).
constexpr double limitInfDeg = 91.0;
constexpr double limitSupDeg = 269.0;

// ── Ganancias PID y perfil de velocidad ──────────────────────────────────
constexpr float kp = 2.0f;
constexpr float ki = 0.2f;
constexpr float kd = 1.2f;

constexpr float maxVelDegS      = 150.0f;
constexpr float cruiseVelDegS   = 50.0f;
constexpr float approachVelDegS = 50.0f;
constexpr float minVelDegS      = 40.0f;

constexpr float tolDeg          = 0.15f;
constexpr float slowZoneDeg     = 5.0f;
constexpr float approachZoneDeg = 15.0f;

constexpr float integralMax = 100.0f;

// ── Central (destino ESP-NOW de esta articulación) ─────────────────────
constexpr uint8_t centralMac[6] = {0x1C, 0xDB, 0xD4, 0x5C, 0xA9, 0xF8};

}  // namespace jointConfig
