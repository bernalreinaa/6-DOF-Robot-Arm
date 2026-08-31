#include "ConveyorBus.h"
#include <esp_now.h>
#include "CentralConfig.h"

void ConveyorBus::begin() {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, centralConfig::conveyorMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("Cinta transportadora registrada como peer ESP-NOW");
    } else {
        Serial.println("Error registrando Cinta transportadora como peer");
    }
}

void ConveyorBus::sendCommand() {
    ConveyorCommand tx{};
    tx.velocityPercent       = velocityPercent_;
    tx.run                   = run_;
    tx.detectionThresholdCm  = detectionThresholdCm_;
    esp_err_t result = esp_now_send(centralConfig::conveyorMac, (uint8_t*)&tx, sizeof(tx));
    if (result != ESP_OK) {
        Serial.println("ERROR al enviar comando a la cinta transportadora");
    }
}

void ConveyorBus::onStatusReceived(const ConveyorStatus& status) {
    objectDetected_ = status.objectDetected;
    lastDistanceCm_ = status.distanceCm;
}
