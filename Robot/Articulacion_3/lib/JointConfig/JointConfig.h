// JointConfig — ÚNICO fichero que cambia entre las 6 articulaciones.
// Todo el resto del firmware (lib/ + src/main.cpp) es idéntico; solo estos
// valores (mecánica, ganancias PID, perfil de velocidad y zona prohibida)
// distinguen una articulación de otra.
#pragma once
#include <Arduino.h>

namespace jointConfig {

constexpr int id = 3;

// ── Pines (iguales en las 6 placas ESP32-C3) ────────────────────────────
constexpr int i2cSdaPin = 5;
constexpr int i2cSclPin = 6;
constexpr int pinDir    = 4;
constexpr int pinPul    = 3;
constexpr int pinEnable = 8;

// ── Mecánica: reductora, microstepping y sentido ────────────────────────
constexpr float ratio = 7.0f;               // Reductora articulación 3
constexpr int   driverMicrosteps = 4;
constexpr int   motorStepsPerRev = 200;
constexpr bool  invertDir = true;           // el error aumenta con polaridad normal en esta articulación

// ── Encoder AS5600: mide directamente el eje del motor (sin gear propio) ──
constexpr bool  encoderOnOutputSide = false;
constexpr float encoderGearRatio = 1.0f;    // sin uso (encoderOnOutputSide == false)

// ── Compensación de backlash (holgura de reductora) ─────────────────────
// Medida experimentalmente: la holgura mecánica de esta articulación ya se
// eliminó/corrigió físicamente, así que queda desactivada (0.0). Dejar la
// posibilidad aquí documentada por si se necesita reactivar en el futuro.
constexpr float backlashOutDeg = 0.0f;

// ── Zona prohibida (grados de SALIDA, 0-360) ────────────────────────────
// Zona prohibida entre 71° y 269° -> rango útil (269°,360°) U (0°,71°) ≈ 162°.
constexpr double limitInfDeg = 71.0;
constexpr double limitSupDeg = 269.0;

// ── Ganancias PID y perfil de velocidad ──────────────────────────────────
constexpr float kp = 4.0f;
constexpr float ki = 0.2f;
constexpr float kd = 2.0f;

constexpr float maxVelDegS      = 150.0f;
constexpr float cruiseVelDegS   = 100.0f;
constexpr float approachVelDegS = 80.0f;
constexpr float minVelDegS      = 20.0f;

constexpr float tolDeg          = 0.1f;
constexpr float slowZoneDeg     = 5.0f;
constexpr float approachZoneDeg = 15.0f;

constexpr float integralMax = 100.0f;

// ── Central (destino ESP-NOW de esta articulación) ─────────────────────
constexpr uint8_t centralMac[6] = {0x1C, 0xDB, 0xD4, 0x5C, 0xA9, 0xF8};

}  // namespace jointConfig
