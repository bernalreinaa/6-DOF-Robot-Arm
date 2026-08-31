// NanoLink — enlace serie con el microcontrolador secundario del mando que
// lee el encoder rotatorio, los pulsadores físicos (SUBIR/BAJAR/IZQUIERDA/
// DERECHA) y la seta de emergencia, y los reporta por esta misma UART como
// líneas de texto ("POS:", "BTN:OK", "SETA:", "HOMEGO", "HOME", "STEP:").
//
// El envío usa un mutex porque más de una tarea del mando puede escribir en
// esta UART (p.ej. al cambiar de página o al ajustar el paso), aunque ambas
// vivan en el mismo núcleo — FreeRTOS puede intercalarlas igualmente.
#pragma once
#include <Arduino.h>

class NanoLink {
public:
    void begin(HardwareSerial& serialPort, uint32_t baudRate, int rxPin, int txPin);

    // Envío protegido por mutex (best-effort: si no consigue el mutex en
    // 20 ms, descarta el mensaje en vez de bloquear la tarea llamante).
    void send(const String& message);

    bool available();

    // Lee una línea completa (hasta '\n'), recortada de espacios. Vacía si
    // no había datos disponibles cuando se llamó.
    String readLine(unsigned long timeoutMs = 10);

private:
    HardwareSerial* serial_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
};
