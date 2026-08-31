// MandoOta — actualización de firmware por WiFi (OTA) bajo demanda.
//
// Ventana de 30s al arrancar: monta un punto de acceso WiFi propio
// ("Mando_OTA") durante 30s; si te conectas a esa red dentro de esos 30s,
// se queda esperando la subida del firmware (con un tope de seguridad de
// 10 min de inactividad). Si no te conectas a tiempo, arranca en modo
// normal (ESP-NOW). Deliberadamente BLOQUEANTE (igual que en las
// articulaciones): mientras se actualiza, el mando no debe enviar
// setpoints ni relevar el estado de su seta remota — y de hecho no puede,
// porque al pasar a WIFI_AP se pierde el ESP-NOW.
#pragma once
#include <Arduino.h>

class MandoOta {
public:
    void bootWindowCheck();
    void enterOtaMode();
};
