// MandoBus — comunicación ESP-NOW del Central con el Mando físico: reporte
// periódico de ángulos hacia el mando (para su pantalla Nextion).
#pragma once
#include <Arduino.h>
#include "EspNowProtocol.h"

class MandoBus {
public:
    void begin();  // registra el Mando como peer ESP-NOW

    // Envía el ángulo actual de las 6 articulaciones al Mando (un paquete
    // AngleMessage por articulación). Llamar periódicamente desde loop()
    // (el propio Central decide el intervalo, p.ej. cada 100 ms).
    void reportAngles(const float angleDeg[6]);
};
