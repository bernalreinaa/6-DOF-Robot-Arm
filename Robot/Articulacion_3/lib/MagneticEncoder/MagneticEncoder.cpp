#include "MagneticEncoder.h"
#include <Wire.h>
#include <AS5600.h>

namespace {
AS5600 sensor;  // instancia única del driver AS5600 (una por articulación)
}

void MagneticEncoder::begin(int sdaPin, int sclPin, uint32_t i2cClockHz) {
    Wire.begin(sdaPin, sclPin, i2cClockHz);
    sensor.begin();
}

bool MagneticEncoder::isConnected() {
    return sensor.isConnected() != 0;
}

void MagneticEncoder::setDirectionCounterClockwise() {
    sensor.setDirection(AS5600_COUNTERCLOCK_WISE);
}

double MagneticEncoder::pollAccumulatedDegrees() {
    double rawModDeg = sensor.readAngle() * 360.0 / 4096.0;  // 0..360

    if (firstSample_) {
        lastRawModDeg_ = rawModDeg;
        firstSample_ = false;
    }

    double delta = rawModDeg - lastRawModDeg_;
    if (delta > 180.0) turns_--;
    else if (delta < -180.0) turns_++;
    lastRawModDeg_ = rawModDeg;

    return turns_ * 360.0 + rawModDeg;
}
