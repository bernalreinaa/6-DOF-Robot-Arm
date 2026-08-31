// JointConfig — ÚNICO fichero que cambia entre las 6 articulaciones.
// Todo el resto del firmware (lib/ + src/main.cpp) es idéntico; solo estos
// valores (mecánica, ganancias PID, perfil de velocidad y zona prohibida)
// distinguen una articulación de otra.
#pragma once
#include <Arduino.h>

namespace jointConfig {

// Identidad de esta articulación (1..6). Se usa para filtrar los mensajes
// ESP-NOW dirigidos a este nodo y para identificarse ante el Central.
constexpr int id = 2;

// ── Pines (iguales en las 6 placas ESP32-C3) ────────────────────────────
constexpr int i2cSdaPin = 5;   // GPIO5 (SDA) — AS5600 y OLED comparten bus I2C
constexpr int i2cSclPin = 6;   // GPIO6 (SCL)
constexpr int pinDir    = 4;   // DIR del TB6600
constexpr int pinPul    = 3;   // PUL (STEP) del TB6600
constexpr int pinEnable = 8;   // ENA del TB6600

// ── Mecánica: reductora, microstepping y sentido ────────────────────────
constexpr float ratio = 71.0f;              // Reductora articulación 2, 71:1
constexpr int   driverMicrosteps = 4;       // Microstepping del TB6600 (1,2,4,8,16,32,64,128)
constexpr int   motorStepsPerRev = 200;     // NEMA17 típico: 1.8°/paso
constexpr bool  invertDir = false;          // true si el error aumenta (polaridad invertida)

// ── Encoder AS5600: ¿mide el eje del motor o va geared al eje de salida? ──
// Articulación "_SENSOR_MODIF": el AS5600 tiene su propio tren de
// engranajes independiente de la reductora del motor. Cadena de salida
// 56 mm que engrana con la de 16 mm que lleva el imán: el sensor gira
// encoderGearRatio veces más rápido que la salida real (56/16 = 3.5).
constexpr bool  encoderOnOutputSide = true;
constexpr float encoderGearRatio = 56.0f / 16.0f;   // 3.5

// ── Compensación de backlash (holgura de reductora) ─────────────────────
// 0.0 = desactivada.
constexpr float backlashOutDeg = 0.0f;

// ── Zona prohibida (grados de SALIDA, 0-360) ────────────────────────────
// Zona prohibida entre 20° y 340° (lado de 180°) -> rango útil ≈ 40° pasando por 0°.
constexpr double limitInfDeg = 35.1;
constexpr double limitSupDeg = 329.9;

// ── Ganancias PID y perfil de velocidad (valores de fábrica; se pueden
//    ajustar en caliente desde la app y persisten en flash — ver JointStorage) ──
constexpr float kp = 4.0f;
constexpr float ki = 0.2f;
constexpr float kd = 2.0f;

constexpr float maxVelDegS      = 40.0f;
constexpr float cruiseVelDegS   = 26.0f;
constexpr float approachVelDegS = 20.0f;
constexpr float minVelDegS      = 10.0f;

constexpr float tolDeg          = 0.05f;
constexpr float slowZoneDeg     = 10.0f;
constexpr float approachZoneDeg = 15.0f;

constexpr float integralMax = 100.0f;   // anti-windup

// ── Central (destino ESP-NOW de esta articulación) ─────────────────────
constexpr uint8_t centralMac[6] = {0x1C, 0xDB, 0xD4, 0x5C, 0xA9, 0xF8};

}  // namespace jointConfig
