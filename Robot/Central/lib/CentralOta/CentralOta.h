// CentralOta — actualización de firmware por WiFi (OTA) bajo demanda.
//
// A diferencia de las articulaciones y el mando, el Central NO puede
// bloquear loop() mientras espera la subida del firmware: tiene que seguir
// atendiendo la seta de emergencia en todo momento por seguridad. Por eso
// enterOtaMode() deja isActive()==true y es loop() quien llama a poll() en
// cada vuelta mientras dure, en vez de un while() bloqueante como en los
// otros firmwares.
#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "StatusBeacon.h"

class CentralOta {
public:
    explicit CentralOta(StatusBeacon& beacon);

    // Ventana de 30s al arrancar (bloqueante, antes de que loop() empiece a
    // correr): si alguien se conecta al WiFi propio del Central
    // ("Central_OTA"), llama a enterOtaMode() y no hace más. Si nadie se
    // conecta, vuelve a modo STA para el arranque normal.
    void bootWindowCheck();

    // Monta el punto de acceso y el servidor OTA; a partir de aquí loop()
    // debe llamar a poll() en cada vuelta hasta que isActive() vuelva a false.
    void enterOtaMode();

    bool isActive() const { return active_; }

    // Atiende el servidor OTA y el timeout de inactividad. Llamar en cada
    // vuelta de loop() mientras isActive() sea true.
    void poll();

private:
    StatusBeacon& beacon_;
    WebServer server_{80};
    bool active_ = false;
    unsigned long startMs_ = 0;
};
