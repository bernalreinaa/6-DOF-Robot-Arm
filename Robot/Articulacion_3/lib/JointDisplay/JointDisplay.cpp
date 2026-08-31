#include "JointDisplay.h"

void JointDisplay::begin() {
    u8g2_.begin();
    u8g2_.clearBuffer();
    u8g2_.setFont(u8g2_font_ncenB08_tr);
}

void JointDisplay::showThreeLines(const char* line1, const char* line2, const char* line3) {
    u8g2_.clearBuffer();
    u8g2_.drawStr(30, 35, line1);
    u8g2_.drawStr(30, 48, line2);
    u8g2_.drawStr(30, 60, line3);
    u8g2_.sendBuffer();
}

void JointDisplay::showStatus(float setpointDeg, float errorDegAbs, float actualDeg, bool lastSendOk) {
    char lineTx[20], lineSp[20], lineErr[20], lineAct[20];
    snprintf(lineTx,  sizeof(lineTx),  "TX: %s", lastSendOk ? "OK" : "FAIL");
    snprintf(lineSp,  sizeof(lineSp),  "SP: %0.2f", setpointDeg);
    snprintf(lineErr, sizeof(lineErr), "Err: %0.2f", errorDegAbs);
    snprintf(lineAct, sizeof(lineAct), "Act: %0.2f", actualDeg);

    u8g2_.clearBuffer();
    u8g2_.drawStr(30, 20, lineTx);
    u8g2_.drawStr(30, 35, lineSp);
    u8g2_.drawStr(30, 48, lineErr);
    u8g2_.drawStr(30, 60, lineAct);
    u8g2_.sendBuffer();
}

void JointDisplay::showOtaCountdown(const String& apName, int secondsLeft) {
    char lineCountdown[16];
    snprintf(lineCountdown, sizeof(lineCountdown), "OTA %ds...", secondsLeft);

    u8g2_.clearBuffer();
    u8g2_.drawStr(10, 10, apName.c_str());
    u8g2_.drawStr(10, 25, lineCountdown);
    u8g2_.drawStr(10, 40, "Conectate para");
    u8g2_.drawStr(10, 52, "actualizar");
    u8g2_.sendBuffer();
}

void JointDisplay::showOtaUploadInfo(const String& apName, const IPAddress& ip) {
    u8g2_.clearBuffer();
    u8g2_.drawStr(10, 15, apName.c_str());
    u8g2_.drawStr(10, 30, ip.toString().c_str());
    u8g2_.drawStr(10, 45, "Actualizando...");
    u8g2_.sendBuffer();
}
