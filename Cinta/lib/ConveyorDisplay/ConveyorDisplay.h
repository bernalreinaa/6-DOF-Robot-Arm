// ConveyorDisplay — envoltorio de la pantalla OLED SSD1306 de 0.42"
// (72x40 px). A diferencia de las articulaciones del brazo (SH1106
// 128x64), este panel es más pequeño: 3 líneas cortas con una fuente
// compacta.
#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

class ConveyorDisplay {
public:
    void begin();

    void showMessage(const char* line1);

    // estado (PARADO / ADEL:NN% / ATRAS:NN% / STOP:OBST), distancia HC-SR04
    // (cm, o -1 si sin lectura) y pasos/segundo realmente aplicados.
    void showStatus(float velocityPercent, float distanceCm, bool obstacleActive, float stepsPerSecond);

private:
    U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2_{U8G2_R0, /* reset=*/ U8X8_PIN_NONE};
};
