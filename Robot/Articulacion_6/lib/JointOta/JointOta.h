// JointOta — actualización de firmware por WiFi (OTA) bajo demanda.
//
// Disparador principal: ventana de 30s al arrancar (bootWindowCheck(),
// llamada desde setup()). Si alguien se conecta al WiFi propio de la
// articulación ("ArtN_OTA") dentro de esos 30s, entra en modo OTA de verdad
// (enterOtaMode(), con 10 min de margen para subir el firmware) y esa
// función ya no vuelve (solo reinicia vía ESP.restart()). Si nadie se
// conecta, arranca en modo normal (ESP-NOW).
// Disparador secundario: escribir "OTA" + Enter en el Monitor Serie durante
// el funcionamiento normal.
// Deliberadamente BLOQUEANTE: mientras se actualiza, la articulación no
// puede aceptar setpoints (al pasar a WIFI_AP se pierde ESP-NOW).
#pragma once
#include <Arduino.h>
#include "JointDisplay.h"

class JointOta {
public:
    JointOta(JointDisplay& display, int jointId);

    void bootWindowCheck();
    void enterOtaMode();

private:
    JointDisplay& display_;
    int jointId_;
    String apName_;
};
