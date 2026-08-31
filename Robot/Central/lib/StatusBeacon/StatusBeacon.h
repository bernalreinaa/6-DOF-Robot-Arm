// StatusBeacon — semáforo (rojo/verde/amarillo) y bocina que indican el
// estado del sistema: rojo = emergencia, amarillo = en movimiento,
// verde = disponible.
#pragma once
#include <Arduino.h>

class StatusBeacon {
public:
    void begin(int pinRed, int pinGreen, int pinYellow, int pinHorn);

    void setColors(bool red, bool green, bool yellow);

    // Actualiza el color según el estado del sistema.
    void update(bool emergencyActive, bool anyJointMoving);

    void beep(int durationMs, int count);

    // Parpadeo del amarillo usado durante la cuenta atrás de la ventana OTA.
    void setYellow(bool on);

private:
    int pinRed_ = -1, pinGreen_ = -1, pinYellow_ = -1, pinHorn_ = -1;
};
