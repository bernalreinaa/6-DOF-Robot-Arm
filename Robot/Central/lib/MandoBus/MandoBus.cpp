#include "MandoBus.h"
#include <esp_now.h>
#include "CentralConfig.h"

void MandoBus::begin() {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, centralConfig::mandoMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("Mando registrado como peer ESP-NOW");
    } else {
        Serial.println("Error registrando Mando como peer");
    }
}

void MandoBus::reportAngles(const float angleDeg[6]) {
    AngleMessage msg;
    for (int i = 1; i <= 6; i++) {
        msg.id = i;
        msg.angleDeg = angleDeg[i - 1];
        esp_now_send(centralConfig::mandoMac, (uint8_t*)&msg, sizeof(msg));
    }
}
