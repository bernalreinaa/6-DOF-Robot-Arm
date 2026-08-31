// EmergencyStop — seta de emergencia física (con antirrebote por
// interrupción) combinada con la seta remota del Mando (recibida por
// ESP-NOW). Activa si CUALQUIERA de las dos está pulsada, para que ninguna
// pueda "liberar" una emergencia que la otra sigue manteniendo activa.
#pragma once
#include <Arduino.h>

class EmergencyStop {
public:
    // pin debe soportar interrupción (CHANGE); se configura INPUT_PULLUP
    // (LOW = pulsada). Llamar DESPUÉS de inicializar WiFi/ESP-NOW: éstos
    // pueden resetear el controlador GPIO y borrar un attachInterrupt previo.
    void begin(int pin);

    // Procesa el antirrebote y recalcula el estado combinado. Llamar en
    // cada vuelta de loop().
    void update();

    // Seta del Mando, recibida por ESP-NOW (ver callback OnDataRecv del
    // Central, comando de emergencia remoto con id=0 centinela).
    void setRemoteActive(bool active) { remoteActive_ = active; }

    bool isActive() const { return active_; }

    // true la primera vez que se consulta tras un cambio de isActive();
    // se resetea automáticamente (patrón "flag de un solo uso").
    bool consumeChangedFlag();

private:
    static void IRAM_ATTR isrTrampoline();
    static EmergencyStop* instance_;

    int  pin_ = -1;
    volatile bool isrFlag_ = false;
    unsigned long debounceStartMs_ = 0;

    bool physicalActive_ = false;
    bool remoteActive_ = false;
    bool active_ = false;
    bool activePrev_ = false;
    bool changedFlag_ = false;
};
