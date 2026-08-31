// ConveyorConfig — pines y parámetros mecánicos de la cinta transportadora.
#pragma once
#include <Arduino.h>

namespace conveyorConfig {

// ── Pines ─────────────────────────────────────────────────────────────
constexpr int i2cSdaPin = 5;   // OLED
constexpr int i2cSclPin = 6;
constexpr int pinDir    = 4;   // DIR del TB6600
constexpr int pinPul    = 7;   // PUL (STEP) del TB6600
constexpr int pinEnable = 8;   // ENA del TB6600 (no conectado físicamente)
constexpr int pinTrig   = 9;   // TRIG del HC-SR04
constexpr int pinEcho   = 10;  // ECHO del HC-SR04

// ── Anchos de pulso/dir del TB6600 (µs) — mismos valores que las articulaciones ──
constexpr uint32_t pulseHighUs = 5;
constexpr uint32_t dirSetupUs  = 5;

// ── Mecánica del motor ───────────────────────────────────────────────────
constexpr int   motorStepsPerRev = 200;   // NEMA17: 1.8°/paso
constexpr int   driverMicrosteps = 4;     // ajustar a los DIP switch reales del TB6600
constexpr float reduction        = 1.0f;  // salida = eje del motor / reduction (1.0 = acople directo)
constexpr float maxSpeedRpm      = 300.0f;  // velocidad del eje de salida al 100%
constexpr bool  invertDir        = true;    // true si la cinta gira al revés de lo esperado

// ── HC-SR04 ───────────────────────────────────────────────────────────────
constexpr unsigned long echoTimeoutUs = 20000UL;   // ~3.4 m, de sobra para una cinta
constexpr int medianSamples = 3;                   // nº de lecturas para el filtro de mediana
constexpr float obstacleDistanceDefaultCm = 4.0f;  // distancia de seguridad de fábrica

// ── Central (destino ESP-NOW) ───────────────────────────────────────────
constexpr uint8_t centralMac[6] = {0x1C, 0xDB, 0xD4, 0x5C, 0xA9, 0xF8};

}  // namespace conveyorConfig
