// ConveyorDrive — control de VELOCIDAD en lazo abierto (sin encoder) del
// motor NEMA17 de la cinta, vía TB6600.
//
// Los pulsos de PASO se generan con un esp_timer (temporizador de
// hardware/software de ESP-IDF) que dispara su propio callback de forma
// periódica y autónoma, INDEPENDIENTE del scheduler de FreeRTOS. Así el
// temporizado del motor no depende de que ninguna tarea "gire" sin ceder
// CPU — ver setVelocityPercent(), que (re)arma ese temporizador con el
// periodo que corresponda, o lo para si la velocidad es ~0.
#pragma once
#include <Arduino.h>
#include <esp_timer.h>

class ConveyorDrive {
public:
    void begin(int pinPul, int pinDir, uint32_t pulseHighUs, uint32_t dirSetupUs,
               float maxSpeedStepsPerSecond, bool invertDir);

    // Arranca/reconfigura/para la generación de pulsos según velocityPercent
    // (-100..100; positivo = adelante, negativo = atrás).
    void setVelocityPercent(float velocityPercent);

    // Pasos/segundo realmente aplicados ahora mismo (0 si está parada).
    float currentStepsPerSecond() const { return currentStepsPerSecond_; }

private:
    int pinPul_ = -1;
    int pinDir_ = -1;
    uint32_t pulseHighUs_ = 5;
    uint32_t dirSetupUs_ = 5;
    bool invertDir_ = false;
    float maxSpeedStepsPerSecond_ = 0.0f;
    float currentStepsPerSecond_ = 0.0f;

    esp_timer_handle_t pulseTimer_ = nullptr;
};
