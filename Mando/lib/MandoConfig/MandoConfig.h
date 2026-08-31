// MandoConfig — pines, direcciones y parámetros del mando físico.
#pragma once
#include <Arduino.h>

namespace mandoConfig {

// MAC del Central (ESP32-S3 central). Se ve en su Serial Monitor al
// arrancar: "MAC del Central: XX:XX:XX:XX:XX:XX".
constexpr uint8_t centralMac[6] = {0x1C, 0xDB, 0xD4, 0x5C, 0xA9, 0xF8};

// ── Pantalla táctil Nextion (UART2) ──────────────────────────────────────
constexpr int nextionTxPin = 17;
constexpr int nextionRxPin = 18;
constexpr uint32_t nextionBaudRate = 9600;
constexpr size_t nextionTxBufferSize = 512;  // margen extra frente a ráfagas (cambio de página, botón OK, etc.)

// ── Enlace serie con el microcontrolador secundario que lee el encoder, los
//    pulsadores y la seta de emergencia física del mando (UART1) ──────────
constexpr uint32_t auxBaudRate = 9600;
constexpr int auxRxPin = 44;
constexpr int auxTxPin = 43;

// ── Botones táctiles capacitivos (ESP32-S3, touchRead) ──────────────────
constexpr int touchPinArt1    = 1;
constexpr int touchPinArt2    = 2;
constexpr int touchPinArt3    = 3;
constexpr int touchPinArt4    = 4;
constexpr int touchPinArt5    = 5;
constexpr int touchPinArt6    = 6;
constexpr int touchPinInicio  = 7;
constexpr int touchPinAjustes = 8;
constexpr int touchPinTareas  = 9;
constexpr int touchThreshold  = 80000;

// ── Velocidad por articulación (%) ───────────────────────────────────────
constexpr float velocityMin  = 5.0f;    // no se permite bajar de este valor
constexpr float velocityMax  = 100.0f;
constexpr float velocityStep = 5.0f;    // paso de los botones +/-, informativo
constexpr float velocityDefault = 10.0f;

}  // namespace mandoConfig
