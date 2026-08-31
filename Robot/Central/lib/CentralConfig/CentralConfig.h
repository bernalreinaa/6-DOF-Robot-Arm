// CentralConfig — pines y direcciones MAC del Central (ESP32-S3, puente
// central de comunicaciones del brazo robótico).
#pragma once
#include <Arduino.h>

namespace centralConfig {

// Direcciones MAC de las 6 articulaciones (ESP32-C3).
constexpr uint8_t jointMac[6][6] = {
    {0xA8, 0x46, 0x74, 0x40, 0x2B, 0xE8},  // Articulación 1
    {0xA8, 0x46, 0x74, 0x40, 0x1B, 0xF0},  // Articulación 2
    {0x50, 0x78, 0x7D, 0xBA, 0xFE, 0x9C},  // Articulación 3
    {0xA8, 0x46, 0x74, 0x40, 0x1D, 0xC8},  // Articulación 4
    {0xA8, 0x46, 0x74, 0x40, 0x49, 0x9C},  // Articulación 5
    {0x10, 0x00, 0x3B, 0xCE, 0x4C, 0x74},  // Articulación 6
};

// MAC del Mando (ESP32-S3 del mando físico). Se ve en su Serial Monitor al
// arrancar: "MAC del Mando: XX:XX:XX:XX:XX:XX".
constexpr uint8_t mandoMac[6] = {0x30, 0xED, 0xA0, 0xB8, 0x86, 0xA4};

// MAC de la cinta transportadora (ESP32-C3). Se ve en su Serial Monitor al
// arrancar: "MAC ESP-NOW: XX:XX:XX:XX:XX:XX".
constexpr uint8_t conveyorMac[6] = {0x08, 0x92, 0x72, 0x8C, 0x55, 0x30};

// ── Multiplexores 74HC4051 (heredados del diseño original; no usados por
//    el firmware actual, que direcciona cada nodo por su MAC ESP-NOW —
//    se conservan como referencia de pines de la placa) ──────────────────
constexpr int muxRxS0 = 10, muxRxS1 = 5, muxRxS2 = 6;
constexpr int muxTxS0 = 1,  muxTxS1 = 2, muxTxS2 = 3;

// ── Baliza (semáforo) y bocina ───────────────────────────────────────────
constexpr int pinRed    = 7;
constexpr int pinGreen  = 15;
constexpr int pinYellow = 16;
constexpr int pinHorn   = 17;

// ── Bomba de vacío ────────────────────────────────────────────────────────
constexpr int pinVacuumPump = 18;

// ── Seta de emergencia (INPUT_PULLUP — LOW = pulsada) ───────────────────
constexpr int pinEmergencyStop = 4;

// Tolerancia para considerar que una articulación sigue en movimiento (°).
constexpr float movementToleranceDeg = 1.5f;

}  // namespace centralConfig
