#include "NextionUi.h"
#include <math.h>

void NextionUi::begin(HardwareSerial& serialPort, uint32_t baudRate, int rxPin, int txPin, size_t txBufferSize) {
    serial_ = &serialPort;
    serial_->setTxBufferSize(txBufferSize);
    serial_->begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void NextionUi::sendTerminator() {
    serial_->write(0xff);
    serial_->write(0xff);
    serial_->write(0xff);
}

void NextionUi::sendFloat(const String& objName, float realValue, int decimals) {
    long intValue = (long)(realValue * pow(10, decimals));
    serial_->print(objName);
    serial_->print(".val=");
    serial_->print(intValue);
    sendTerminator();
}

void NextionUi::updateDegreeBar(const String& objName, int degrees) {
    degrees = constrain(degrees, 0, 360);
    int percent = (int)((degrees * 100.0f) / 360.0f);
    serial_->print(objName);
    serial_->print(".val=");
    serial_->print(percent);
    sendTerminator();
}

void NextionUi::changePage(const String& pageName) {
    serial_->print("page ");
    serial_->print(pageName);
    sendTerminator();
}

bool NextionUi::available() { return serial_->available(); }
int NextionUi::read() { return serial_->read(); }
void NextionUi::setTimeout(unsigned long timeoutMs) { serial_->setTimeout(timeoutMs); }
String NextionUi::readStringUntil(char terminator) { return serial_->readStringUntil(terminator); }
