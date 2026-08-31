#include "CentralLink.h"
#include <esp_now.h>
#include "MandoConfig.h"

CentralLink* CentralLink::instance_ = nullptr;

void CentralLink::begin() {
    instance_ = this;

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mandoConfig::centralMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Error registrando central como peer");
    } else {
        Serial.println("Central registrado OK");
    }

    esp_now_register_recv_cb(onDataRecvTrampoline);
}

void CentralLink::sendSetpoint(int jointId, float angleDeg, float velocityPercent) {
    if (jointId < 1 || jointId > 6) return;
    SetpointCommand tx{};
    tx.id               = jointId;
    tx.setpointDeg       = angleDeg;
    tx.reset             = false;
    tx.motorDisabled     = false;
    tx.velocityPercent   = velocityPercent;
    esp_now_send(mandoConfig::centralMac, (uint8_t*)&tx, sizeof(tx));
}

void CentralLink::sendReset(int jointId) {
    if (jointId < 1 || jointId > 6) return;
    SetpointCommand tx{};
    tx.id               = jointId;
    tx.setpointDeg       = 0.0f;  // ignorado por la articulación cuando reset=true
    tx.reset             = true;
    tx.motorDisabled     = false;
    tx.velocityPercent   = 100.0f;  // no aplica a un reset, valor neutro
    esp_now_send(mandoConfig::centralMac, (uint8_t*)&tx, sizeof(tx));
}

void CentralLink::sendEmergency(bool pressed) {
    SetpointCommand tx{};
    tx.id               = 0;        // centinela: comando de emergencia, no un setpoint
    tx.setpointDeg       = 0.0f;
    tx.reset             = false;
    tx.motorDisabled     = pressed;  // true = activar emergencia, false = liberar
    tx.velocityPercent   = 100.0f;   // no aplica, valor neutro

    for (int i = 0; i < 3; i++) {
        esp_now_send(mandoConfig::centralMac, (uint8_t*)&tx, sizeof(tx));
        if (i < 2) vTaskDelay(pdMS_TO_TICKS(5));
    }
}

float CentralLink::lastAngleDeg(int jointId) const {
    if (jointId < 1 || jointId > 6) return 0.0f;
    return angleDeg_[jointId - 1];
}

void CentralLink::onDataRecvTrampoline(const uint8_t* mac, const uint8_t* data, int len) {
    if (instance_) instance_->onDataRecv(data, len);
}

void CentralLink::onDataRecv(const uint8_t* data, int len) {
    if (len == sizeof(AngleMessage)) {
        AngleMessage msg;
        memcpy(&msg, data, sizeof(msg));
        if (msg.id >= 1 && msg.id <= 6) {
            angleDeg_[msg.id - 1] = msg.angleDeg;
        }
    }
}
