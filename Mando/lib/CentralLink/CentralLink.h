// CentralLink — comunicación ESP-NOW del Mando con el Central central:
// envío de setpoints/reset/emergencia y recepción de los ángulos actuales
// de las 6 articulaciones.
#pragma once
#include <Arduino.h>
#include "EspNowProtocol.h"

class CentralLink {
public:
    void begin();  // registra el Central como peer y el callback de recepción

    // Envía un setpoint a una articulación (1-6). Se llama solo desde la
    // tarea que atiende la Nextion/UART auxiliar (nunca hay dos llamadas a
    // esp_now_send() concurrentes entre sí).
    void sendSetpoint(int jointId, float angleDeg, float velocityPercent);

    // Recalibra el offset de una articulación a su posición física actual
    // (no la mueve).
    void sendReset(int jointId);

    // Réplica de la seta de emergencia hacia el Central (id=0 centinela).
    // Envío triplicado: señal crítica de seguridad y ESP-NOW no tiene ACK a
    // nivel de aplicación.
    void sendEmergency(bool pressed);

    // Último ángulo recibido por articulación (1-6); 0.0 si aún no se ha
    // recibido ninguno.
    float lastAngleDeg(int jointId) const;

private:
    static void onDataRecvTrampoline(const uint8_t* mac, const uint8_t* data, int len);
    void onDataRecv(const uint8_t* data, int len);

    static CentralLink* instance_;
    volatile float angleDeg_[6] = {0, 0, 0, 0, 0, 0};
};
