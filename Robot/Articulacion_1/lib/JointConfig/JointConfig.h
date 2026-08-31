// JointConfig — ÚNICO fichero que cambia entre las 6 articulaciones.
// Todo el resto del firmware (lib/ + src/main.cpp) es idéntico; solo estos
// valores (mecánica, ganancias PID, perfil de velocidad y zona prohibida)
// distinguen una articulación de otra.
#pragma once
#include <Arduino.h>

namespace jointConfig {

// Identidad de esta articulación (1..6). Se usa para filtrar los mensajes
// ESP-NOW dirigidos a este nodo y para identificarse ante el Central.
constexpr int id = 1;

// ── Pines (iguales en las 6 placas ESP32-C3) ────────────────────────────
constexpr int i2cSdaPin = 5;   // GPIO5 (SDA) — AS5600 y OLED comparten bus I2C
constexpr int i2cSclPin = 6;   // GPIO6 (SCL)
constexpr int pinDir    = 4;   // DIR del TB6600
constexpr int pinPul    = 3;   // PUL (STEP) del TB6600
constexpr int pinEnable = 8;   // ENA del TB6600

// ── Mecánica: reductora, microstepping y sentido ────────────────────────
// Reductora motor->salida. Si se cambia la reductora física, solo tocar esto.
constexpr float ratio = 8.0f;               // Motor 1:50; Polea Ø9.6mm / Ø37.7mm
constexpr int   driverMicrosteps = 4;       // Microstepping del TB6600 (1,2,4,8,16,32,64,128)
constexpr int   motorStepsPerRev = 200;     // NEMA17 típico: 1.8°/paso
constexpr bool  invertDir = false;          // true si el error aumenta (polaridad invertida)

// ── Encoder AS5600: ¿mide el eje del motor o va geared al eje de salida? ──
// false (mayoría de articulaciones): el imán del AS5600 está en el propio
//   eje del motor, ANTES de la reductora — su lectura hay que dividirla por
//   "ratio" para obtener el ángulo de salida.
// true ("_SENSOR_MODIF"): el AS5600 tiene su propio tren de engranajes
//   independiente, geared directamente a la salida — su lectura hay que
//   dividirla por "encoderGearRatio" (no por "ratio") para obtener el
//   ángulo de salida.
constexpr bool  encoderOnOutputSide = false;
constexpr float encoderGearRatio = 1.0f;    // solo se usa si encoderOnOutputSide == true

// ── Compensación de backlash (holgura de reductora) ─────────────────────
// Al cambiar de sentido, la salida no se mueve hasta que el motor recorre
// este ángulo de "zona muerta" (medido en la SALIDA). 0.0 = desactivada.
constexpr float backlashOutDeg = 0.0f;

// ── Zona prohibida (grados de SALIDA, 0-360) ────────────────────────────
// Regla: si la zona prohibida cruza el 0°, limitInfDeg > limitSupDeg. Si no
// cruza el 0°, limitInfDeg < limitSupDeg. limitInfDeg == limitSupDeg = sin
// zona prohibida.
// Zona prohibida entre 90° y 270° -> rango útil (270°,360°) U (0°,90°) = 180°.
constexpr double limitInfDeg = 90.0;
constexpr double limitSupDeg = 270.0;

// ── Ganancias PID y perfil de velocidad (valores de fábrica; se pueden
//    ajustar en caliente desde la app y persisten en flash — ver JointStorage) ──
constexpr float kp = 3.0f;
constexpr float ki = 0.2f;
constexpr float kd = 0.0f;

constexpr float maxVelDegS      = 400.0f;
constexpr float cruiseVelDegS   = 250.0f;
constexpr float approachVelDegS = 150.0f;
constexpr float minVelDegS      = 40.0f;

constexpr float tolDeg          = 0.5f;
constexpr float slowZoneDeg     = 5.0f;
constexpr float approachZoneDeg = 15.0f;

constexpr float integralMax = 100.0f;   // anti-windup

// ── Central (destino ESP-NOW de esta articulación) ─────────────────────
constexpr uint8_t centralMac[6] = {0x1C, 0xDB, 0xD4, 0x5C, 0xA9, 0xF8};

}  // namespace jointConfig
