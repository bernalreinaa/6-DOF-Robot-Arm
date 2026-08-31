#include "NanoLink.h"

void NanoLink::begin(HardwareSerial& serialPort, uint32_t baudRate, int rxPin, int txPin) {
    serial_ = &serialPort;
    mutex_ = xSemaphoreCreateMutex();
    serial_->begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void NanoLink::send(const String& message) {
    if (mutex_ && xSemaphoreTake(mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        serial_->print(message);
        xSemaphoreGive(mutex_);
    }
}

bool NanoLink::available() {
    return serial_->available();
}

String NanoLink::readLine(unsigned long timeoutMs) {
    serial_->setTimeout(timeoutMs);
    String line = serial_->readStringUntil('\n');
    line.trim();
    return line;
}
