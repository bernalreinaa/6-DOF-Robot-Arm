// JointDisplay — envoltorio de la pantalla OLED SH1106 (128x64, I2C) que
// muestra el estado en tiempo real de la articulación.
#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

class JointDisplay {
public:
    // Debe llamarse después de Wire.begin() (bus I2C ya inicializado por
    // MagneticEncoder::begin(), que comparte el mismo bus).
    void begin();

    void showThreeLines(const char* line1, const char* line2, const char* line3);

    // Estado normal en marcha: setpoint / error / ángulo actual / enlace TX.
    void showStatus(float setpointDeg, float errorDegAbs, float actualDeg, bool lastSendOk);

    // Ventana OTA de arranque, con cuenta atrás.
    void showOtaCountdown(const String& apName, int secondsLeft);

    // Modo OTA activo, a la espera de la subida del firmware.
    void showOtaUploadInfo(const String& apName, const IPAddress& ip);

private:
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_{U8G2_R0, /* reset=*/ U8X8_PIN_NONE};
};
