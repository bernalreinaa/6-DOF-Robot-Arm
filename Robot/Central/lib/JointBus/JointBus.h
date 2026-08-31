// JointBus — comunicación ESP-NOW del Central con las 6 articulaciones:
// envío de setpoints/reset/enable, envío y recepción de parámetros de ajuste
// (PID/velocidad/zonas), y el estado (ángulos) que reportan continuamente.
#pragma once
#include <Arduino.h>
#include "EspNowProtocol.h"
#include "CentralConfig.h"

class JointBus {
public:
    void begin();  // registra las 6 articulaciones como peers ESP-NOW

    // --- Setpoints ---
    // Fija el ángulo deseado para jointId (1..6); no envía nada todavía
    // (ver flushPendingSetpoints).
    void setTargetAngle(int jointId, float deg);
    float targetAngle(int jointId) const;

    // Envía por ESP-NOW el setpoint de cada articulación cuyo valor cambió
    // desde el último envío (evita mandar comandos a nodos que no deben
    // moverse). velocityPercent se aplica a todas las que se envíen.
    void flushPendingSetpoints(float velocityPercent);

    // Recalibra el offset de una articulación a su posición física actual
    // (no la mueve). preserveEnabled = mantener su estado enable/disable actual.
    void sendReset(int jointId, bool preserveEnabled = false);

    // Alterna el estado enable/disable de una articulación.
    void toggleEnable(int jointId, float velocityPercent);

    // Deshabilita (o rehabilita) las 6 articulaciones a la vez — parada de
    // emergencia. El setpoint enviado es el actual (no mueve nada porque el
    // flag "disabled" bloquea el motor en la propia articulación).
    void setAllDisabled(bool disabled);

    bool isDisabled(int jointId) const;

    // --- Ajuste de parámetros (PID / velocidad / zonas) ---
    void sendTuning(int jointId, const TuningParams& params);
    void sendReload(int jointId);
    // Línea "reload[i]=kp;ki;...;limitSup;" con los últimos parámetros
    // conocidos de esa articulación, para reenviar al PC.
    String formatTuningReport(int jointId) const;

    // --- Recepción (llamar desde el callback ESP-NOW del Central) ---
    void onAngleReceived(const AngleMessage& msg);
    void onTuningReceived(const TuningParams& msg);

    float angleDeg(int jointId) const;
    bool isAnyJointMoving() const;

private:
    float angleDeg_[6]          = {0, 0, 0, 0, 0, 0};
    float targetAngleDeg_[6]    = {0, 0, 0, 0, 0, 0};
    float lastSentAngleDeg_[6]  = {-9999, -9999, -9999, -9999, -9999, -9999};
    bool  disabled_[6]          = {false, false, false, false, false, false};
    TuningParams lastTuning_[6] = {};

    void sendSetpointPacket(int jointId, float setpointDeg, bool reset, bool disabled, float velocityPercent);
};
