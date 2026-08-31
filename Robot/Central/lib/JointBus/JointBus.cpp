#include "JointBus.h"
#include <esp_now.h>
#include <math.h>

void JointBus::begin() {
    for (int i = 0; i < 6; i++) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, centralConfig::jointMac[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }
}

void JointBus::setTargetAngle(int jointId, float deg) {
    if (jointId < 1 || jointId > 6) return;
    targetAngleDeg_[jointId - 1] = deg;
}

float JointBus::targetAngle(int jointId) const {
    if (jointId < 1 || jointId > 6) return 0.0f;
    return targetAngleDeg_[jointId - 1];
}

void JointBus::sendSetpointPacket(int jointId, float setpointDeg, bool reset, bool disabled, float velocityPercent) {
    SetpointCommand tx{};
    tx.id               = jointId;
    tx.setpointDeg       = setpointDeg;
    tx.reset             = reset;
    tx.motorDisabled     = disabled;
    tx.velocityPercent   = velocityPercent;
    esp_now_send(centralConfig::jointMac[jointId - 1], (uint8_t*)&tx, sizeof(tx));
}

void JointBus::flushPendingSetpoints(float velocityPercent) {
    for (int i = 0; i < 6; i++) {
        if (fabs(targetAngleDeg_[i] - lastSentAngleDeg_[i]) < 0.01f) continue;
        sendSetpointPacket(i + 1, targetAngleDeg_[i], false, disabled_[i], velocityPercent);
        lastSentAngleDeg_[i] = targetAngleDeg_[i];
        Serial.printf("Enviado %.2f al Nodo %d\n", targetAngleDeg_[i], i + 1);
    }
}

void JointBus::sendReset(int jointId, bool preserveEnabled) {
    if (jointId < 1 || jointId > 6) return;
    bool disabled = preserveEnabled ? disabled_[jointId - 1] : false;
    sendSetpointPacket(jointId, 0.0f, true, disabled, 100.0f);  // reset siempre a velocidad normal
    Serial.printf(">>RESET enviado a articulacion %d\n", jointId);
}

void JointBus::toggleEnable(int jointId, float velocityPercent) {
    if (jointId < 1 || jointId > 6) return;
    disabled_[jointId - 1] = !disabled_[jointId - 1];
    // Evitar setpoint/reset residual de otra operación: se reenvía el
    // objetivo actual de esta articulación, sin reset.
    sendSetpointPacket(jointId, targetAngleDeg_[jointId - 1], false, disabled_[jointId - 1], velocityPercent);
    Serial.printf(">>ENABLE enviado a articulacion %d\n", jointId);
}

void JointBus::setAllDisabled(bool disabled) {
    for (int i = 0; i < 6; i++) {
        disabled_[i] = disabled;
        sendSetpointPacket(i + 1, targetAngleDeg_[i], false, disabled, 100.0f);  // no mueve, el flag bloquea el motor
        lastSentAngleDeg_[i] = targetAngleDeg_[i];  // marcar como enviado para no reenviar en el siguiente flush
    }
}

bool JointBus::isDisabled(int jointId) const {
    if (jointId < 1 || jointId > 6) return false;
    return disabled_[jointId - 1];
}

void JointBus::sendTuning(int jointId, const TuningParams& params) {
    if (jointId < 1 || jointId > 6) return;
    lastTuning_[jointId - 1] = params;
    esp_err_t result = esp_now_send(centralConfig::jointMac[jointId - 1], (uint8_t*)&params, sizeof(params));
    Serial.println(result == ESP_OK
                        ? ("OK Enviado al Nodo " + String(jointId) + " los valores de ajuste")
                        : ("ERROR al enviar al Nodo " + String(jointId) + " los valores de ajuste"));
}

void JointBus::sendReload(int jointId) {
    if (jointId < 1 || jointId > 6) return;
    ReloadCommand reload{true};
    esp_now_send(centralConfig::jointMac[jointId - 1], (uint8_t*)&reload, sizeof(reload));
}

String JointBus::formatTuningReport(int jointId) const {
    if (jointId < 1 || jointId > 6) return "";
    const TuningParams& t = lastTuning_[jointId - 1];
    String msg = "reload[" + String(jointId) + "]=";
    msg += String(t.kp) + ";";
    msg += String(t.ki) + ";";
    msg += String(t.kd) + ";";
    msg += String(t.maxVelDegS) + ";";
    msg += String(t.cruiseVelDegS) + ";";
    msg += String(t.approachVelDegS) + ";";
    msg += String(t.minVelDegS) + ";";
    msg += String(t.tolDeg) + ";";
    msg += String(t.slowZoneDeg) + ";";
    msg += String(t.approachZoneDeg) + ";";
    msg += String(t.limitInfDeg) + ";";
    msg += String(t.limitSupDeg) + ";";
    return msg;
}

void JointBus::onAngleReceived(const AngleMessage& msg) {
    if (msg.id >= 1 && msg.id <= 6) {
        angleDeg_[msg.id - 1] = msg.angleDeg;
    }
}

void JointBus::onTuningReceived(const TuningParams& msg) {
    if (msg.id >= 1 && msg.id <= 6) {
        lastTuning_[msg.id - 1] = msg;
    }
}

float JointBus::angleDeg(int jointId) const {
    if (jointId < 1 || jointId > 6) return 0.0f;
    return angleDeg_[jointId - 1];
}

bool JointBus::isAnyJointMoving() const {
    for (int i = 0; i < 6; i++) {
        float diff = fmod(fabs(angleDeg_[i] - targetAngleDeg_[i]), 360.0f);
        if (diff > 180.0f) diff = 360.0f - diff;
        if (diff > centralConfig::movementToleranceDeg) return true;
    }
    return false;
}
