// StepperDriver — envoltorio del driver TB6600 (motor paso a paso NEMA17):
// generación de pulsos PUL/DIR y control de la señal ENA.
#pragma once
#include <Arduino.h>

class StepperDriver {
public:
    void begin(int pinPul, int pinDir, int pinEnable);

    // NOTA: por el cableado del TB6600 (ENA activo en HIGH = motor
    // deshabilitado), "disabled=true" pone el pin ENA en HIGH.
    void setDisabled(bool disabled);

    // Genera un único pulso de paso en la dirección indicada (helper de alto
    // nivel para movimientos simples, temporización fija).
    void pulse(bool directionForward, uint32_t pulseHighUs, uint32_t pulseLowUs);

    // Fija DIR sin generar pulso, respetando el tiempo de setup del driver.
    void setDirection(bool directionForward, uint32_t dirSetupUs);

    // Primitivas de bajo nivel usadas por JointMotionController para
    // controlar con precisión el intervalo entre pulsos (velocidad variable
    // según el lazo PID): flanco de subida y de bajada de PUL por separado.
    void stepHigh(uint32_t highUs);
    void stepLow(uint32_t lowUs);

private:
    int pinPul_ = -1;
    int pinDir_ = -1;
    int pinEnable_ = -1;
};
