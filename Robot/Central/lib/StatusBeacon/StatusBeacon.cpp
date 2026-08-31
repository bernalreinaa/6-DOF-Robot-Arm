#include "StatusBeacon.h"

void StatusBeacon::begin(int pinRed, int pinGreen, int pinYellow, int pinHorn) {
    pinRed_ = pinRed;
    pinGreen_ = pinGreen;
    pinYellow_ = pinYellow;
    pinHorn_ = pinHorn;
    pinMode(pinRed_, OUTPUT);
    pinMode(pinGreen_, OUTPUT);
    pinMode(pinYellow_, OUTPUT);
    pinMode(pinHorn_, OUTPUT);
}

void StatusBeacon::setColors(bool red, bool green, bool yellow) {
    digitalWrite(pinRed_,    red    ? HIGH : LOW);
    digitalWrite(pinGreen_,  green  ? HIGH : LOW);
    digitalWrite(pinYellow_, yellow ? HIGH : LOW);
}

void StatusBeacon::update(bool emergencyActive, bool anyJointMoving) {
    if (emergencyActive) {
        setColors(true, false, false);   // ROJO — emergencia
    } else if (anyJointMoving) {
        setColors(false, false, true);   // AMARILLO — en movimiento
    } else {
        setColors(false, true, false);   // VERDE — disponible
    }
}

void StatusBeacon::beep(int durationMs, int count) {
    for (int i = 0; i < count; i++) {
        digitalWrite(pinHorn_, HIGH);
        delay(durationMs);
        digitalWrite(pinHorn_, LOW);
        delay(durationMs);
    }
}

void StatusBeacon::setYellow(bool on) {
    digitalWrite(pinYellow_, on ? HIGH : LOW);
}
