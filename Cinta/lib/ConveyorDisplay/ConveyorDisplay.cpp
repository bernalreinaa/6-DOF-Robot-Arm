#include "ConveyorDisplay.h"
#include <math.h>

void ConveyorDisplay::begin() {
    u8g2_.begin();
    u8g2_.clearBuffer();
    u8g2_.setFont(u8g2_font_6x10_tf);
}

void ConveyorDisplay::showMessage(const char* line1) {
    u8g2_.clearBuffer();
    u8g2_.setFont(u8g2_font_6x10_tf);
    u8g2_.drawStr(2, 20, line1);
    u8g2_.sendBuffer();
}

void ConveyorDisplay::showStatus(float velocityPercent, float distanceCm, bool obstacleActive, float stepsPerSecond) {
    char line1[16], line2[16], line3[16];

    if (obstacleActive) {
        snprintf(line1, sizeof(line1), "STOP:OBST");
    } else if (fabs(velocityPercent) < 0.5f) {
        snprintf(line1, sizeof(line1), "PARADO");
    } else {
        snprintf(line1, sizeof(line1), "%s:%.0f%%",
                  velocityPercent > 0 ? "ADEL" : "ATRAS", fabs(velocityPercent));
    }

    if (distanceCm < 0.0f) {
        snprintf(line2, sizeof(line2), "D: ---");
    } else {
        snprintf(line2, sizeof(line2), "D:%.1fcm", distanceCm);
    }

    snprintf(line3, sizeof(line3), "%.0fp/s", stepsPerSecond);

    u8g2_.clearBuffer();
    u8g2_.setFont(u8g2_font_6x10_tf);
    u8g2_.drawStr(2, 11, line1);
    u8g2_.drawStr(2, 24, line2);
    u8g2_.drawStr(2, 37, line3);
    u8g2_.sendBuffer();
}
