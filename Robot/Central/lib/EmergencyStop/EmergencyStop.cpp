#include "EmergencyStop.h"

namespace {
constexpr unsigned long kDebounceMs = 50;
}

EmergencyStop* EmergencyStop::instance_ = nullptr;

void IRAM_ATTR EmergencyStop::isrTrampoline() {
    if (instance_) instance_->isrFlag_ = true;
}

void EmergencyStop::begin(int pin) {
    pin_ = pin;
    instance_ = this;
    pinMode(pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_), isrTrampoline, CHANGE);
}

void EmergencyStop::update() {
    if (isrFlag_) {
        isrFlag_ = false;
        debounceStartMs_ = millis();  // arrancar temporizador de debounce
    }
    if (debounceStartMs_ != 0 && (millis() - debounceStartMs_ >= kDebounceMs)) {
        debounceStartMs_ = 0;
        physicalActive_ = (digitalRead(pin_) == LOW);  // LOW = pulsada (pull-up)
    }

    active_ = physicalActive_ || remoteActive_;

    if (active_ != activePrev_) {
        activePrev_ = active_;
        changedFlag_ = true;
    }
}

bool EmergencyStop::consumeChangedFlag() {
    if (!changedFlag_) return false;
    changedFlag_ = false;
    return true;
}
