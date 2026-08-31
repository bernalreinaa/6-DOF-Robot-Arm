#include "StepperDriver.h"

void StepperDriver::begin(int pinPul, int pinDir, int pinEnable) {
    pinPul_ = pinPul;
    pinDir_ = pinDir;
    pinEnable_ = pinEnable;
    pinMode(pinPul_, OUTPUT);
    pinMode(pinDir_, OUTPUT);
    pinMode(pinEnable_, OUTPUT);
    setDisabled(true);  // arranca deshabilitado, igual que el firmware original
}

void StepperDriver::setDisabled(bool disabled) {
    digitalWrite(pinEnable_, disabled ? HIGH : LOW);
}

void StepperDriver::setDirection(bool directionForward, uint32_t dirSetupUs) {
    digitalWrite(pinDir_, directionForward ? LOW : HIGH);
    delayMicroseconds(dirSetupUs);
}

void StepperDriver::pulse(bool directionForward, uint32_t pulseHighUs, uint32_t pulseLowUs) {
    digitalWrite(pinDir_, directionForward ? LOW : HIGH);
    digitalWrite(pinPul_, HIGH);
    delayMicroseconds(pulseHighUs);
    digitalWrite(pinPul_, LOW);
    delayMicroseconds(pulseLowUs);
}

void StepperDriver::stepHigh(uint32_t highUs) {
    digitalWrite(pinPul_, HIGH);
    delayMicroseconds(highUs);
}

void StepperDriver::stepLow(uint32_t lowUs) {
    digitalWrite(pinPul_, LOW);
    delayMicroseconds(lowUs);
}
