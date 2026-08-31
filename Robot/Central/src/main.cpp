//------------------------------------------------------------------------
//  Central (ESP32-S3) — puente central de comunicaciones del brazo
//  robótico de 6 GDL: recibe por Serial los comandos del PC (Python) y los
//  distribuye por ESP-NOW a las 6 articulaciones, la cinta transportadora y
//  el mando físico; agrega y reenvía al PC el estado de todos ellos.
//
//  Este fichero es el punto de orquestación: crea los objetos de las
//  librerías (lib/), registra los peers ESP-NOW y conecta el callback de
//  recepción con JointBus/MandoBus/ConveyorBus. TODA la lógica de cada
//  subsistema vive en su librería — ver lib/JointBus, lib/MandoBus,
//  lib/ConveyorBus, lib/EmergencyStop, lib/StatusBeacon, lib/CentralOta y
//  lib/EspNowProtocol / lib/ConveyorProtocol.
//------------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "CentralConfig.h"
#include "EspNowProtocol.h"
#include "ConveyorProtocol.h"
#include "JointBus.h"
#include "MandoBus.h"
#include "ConveyorBus.h"
#include "EmergencyStop.h"
#include "StatusBeacon.h"
#include "CentralOta.h"

//------------------------------------------------------------------------
//  Objetos de las librerías
//------------------------------------------------------------------------
JointBus jointBus;
MandoBus mandoBus;
ConveyorBus conveyorBus;
EmergencyStop emergencyStop;
StatusBeacon beacon;
CentralOta ota(beacon);

//------------------------------------------------------------------------
//  Estado compartido entre el callback ESP-NOW y loop()
//------------------------------------------------------------------------
// % de velocidad (10-100) para el PRÓXIMO envío de setpoints por Serial,
// fijado por el token opcional "V=NN;" al final de una línea SP[...].
float pcVelocityPercent = 100.0f;

// Setpoints/resets pendientes que llegaron del mando por ESP-NOW. Se
// procesan en loop() (nunca en el callback de recepción) para no llamar
// esp_now_send() desde dentro de OnDataRecv.
volatile bool  mandoSetpointPending[6] = {false, false, false, false, false, false};
volatile float mandoSetpointValue[6]   = {0, 0, 0, 0, 0, 0};
volatile float mandoVelocityPercent[6] = {100, 100, 100, 100, 100, 100};
volatile bool  mandoResetPending[6]    = {false, false, false, false, false, false};

static unsigned long lastAngleReportMs = 0;
const unsigned long ANGLE_REPORT_INTERVAL_MS = 100;
static unsigned long lastMandoReportMs = 0;
const unsigned long MANDO_REPORT_MS = 100;

//------------------------------------------------------------------------
//  ESP-NOW: callback de recepción — discrimina el tipo de paquete por su
//  tamaño (sizeof), igual que el resto del protocolo del proyecto.
//------------------------------------------------------------------------
void onEspNowDataRecv(const uint8_t* /*mac*/, const uint8_t* incomingData, int len) {
    if (len == sizeof(TuningParams)) {
        // Parámetros iniciales que reporta una articulación (al arrancar o
        // tras un reload).
        TuningParams msg;
        memcpy(&msg, incomingData, sizeof(msg));
        jointBus.onTuningReceived(msg);
        Serial.printf("Parametros iniciales guardados para el ID: %d\n", msg.id);

    } else if (len == sizeof(SetpointCommand)) {
        // Setpoint / reset / emergencia remota, siempre proveniente del Mando.
        SetpointCommand msg;
        memcpy(&msg, incomingData, sizeof(msg));

        if (msg.id == 0) {
            // Centinela: comando de emergencia remoto (seta del mando).
            emergencyStop.setRemoteActive(msg.motorDisabled);
        } else if (msg.id >= 1 && msg.id <= 6 && !emergencyStop.isActive() && msg.reset) {
            mandoResetPending[msg.id - 1] = true;
        } else if (msg.id >= 1 && msg.id <= 6 && !emergencyStop.isActive()) {
            mandoSetpointValue[msg.id - 1]   = msg.setpointDeg;
            mandoVelocityPercent[msg.id - 1] = msg.velocityPercent;
            mandoSetpointPending[msg.id - 1] = true;
        }

    } else if (len == sizeof(AngleMessage)) {
        // Ángulo continuo de una articulación.
        AngleMessage msg;
        memcpy(&msg, incomingData, sizeof(msg));
        jointBus.onAngleReceived(msg);

    } else if (len == sizeof(ConveyorStatus)) {
        // Estado de la cinta transportadora (objeto detectado + distancia).
        ConveyorStatus msg;
        memcpy(&msg, incomingData, sizeof(msg));
        conveyorBus.onStatusReceived(msg);
    }
}

//------------------------------------------------------------------------
//  Comandos por Serial (protocolo de texto con el PC / app Python)
//------------------------------------------------------------------------

// "SP[1]=10.0;SP[2]=20.0;...;SP[6]=60.0;V=75;" — el token V=NN es opcional.
void handleSetpointCommand(const String& line) {
    pcVelocityPercent = 100.0f;  // por defecto, salvo que la línea traiga V=NN

    int start = 0;
    while (start < (int)line.length()) {
        int posEnd = line.indexOf(';', start);
        String token = (posEnd == -1) ? line.substring(start) : line.substring(start, posEnd);
        token.trim();
        if (token.length() == 0) break;

        if (token.startsWith("V=")) {
            pcVelocityPercent = constrain(token.substring(2).toFloat(), 10.0f, 100.0f);
        } else {
            int idxL = token.indexOf('[');
            int idxR = token.indexOf(']');
            int idxEq = token.indexOf('=');
            if (idxL != -1 && idxR != -1 && idxEq != -1 && idxR > idxL && idxEq > idxR) {
                int jointId = token.substring(idxL + 1, idxR).toInt();
                float value = token.substring(idxEq + 1).toFloat();
                if (jointId >= 1 && jointId <= 6) jointBus.setTargetAngle(jointId, value);
            }
        }

        if (posEnd == -1) break;
        start = posEnd + 1;
    }
}

// "reset[3]=0.00"
void handleResetCommand(const String& line) {
    int idxL = line.indexOf('[');
    int idxR = line.indexOf(']');
    if (idxL == -1 || idxR == -1 || idxR <= idxL + 1) return;
    int jointId = line.substring(idxL + 1, idxR).toInt();
    if (jointId < 1 || jointId > 6) return;
    jointBus.sendReset(jointId);
}

// "enable[3]=..." (el valor no se usa; alterna el estado)
void handleEnableCommand(const String& line) {
    int idxL = line.indexOf('[');
    int idxR = line.indexOf(']');
    if (idxL == -1 || idxR == -1 || idxR <= idxL + 1) return;
    int jointId = line.substring(idxL + 1, idxR).toInt();
    if (jointId < 1 || jointId > 6) return;
    jointBus.toggleEnable(jointId, pcVelocityPercent);
}

// "init[3]3.4;12.25;...;0.5;0.2;0.1;10.0;5.0;2.0;15.0;10.0" (kp, ki, kd,
// max_vel, cruise_vel, approach_vel, min_vel, tol_deg, slow_zone_deg,
// approach_zone_deg, limit_inf, limit_sup) — sin '=' entre ']' y los valores.
void handleInitCommand(const String& line) {
    int idxL = line.indexOf('[');
    int idxR = line.indexOf(']');
    if (idxL == -1 || idxR == -1 || idxR <= idxL + 1) return;
    int jointId = line.substring(idxL + 1, idxR).toInt();
    if (jointId < 1 || jointId > 6) return;

    Serial.println(">>Comando INIT recibido");
    Serial.println(line);

    TuningParams params{};
    params.id = jointId;

    int start = idxR + 1;
    int paramIndex = 0;
    while (start < (int)line.length() && paramIndex < 12) {
        int posEnd = line.indexOf(';', start);
        String token = (posEnd == -1) ? line.substring(start) : line.substring(start, posEnd);
        token.trim();
        if (token.length() == 0) break;

        float value = token.toFloat();
        switch (paramIndex) {
            case 0:  params.kp = value; break;
            case 1:  params.ki = value; break;
            case 2:  params.kd = value; break;
            case 3:  params.maxVelDegS = value; break;
            case 4:  params.cruiseVelDegS = value; break;
            case 5:  params.approachVelDegS = value; break;
            case 6:  params.minVelDegS = value; break;
            case 7:  params.tolDeg = value; break;
            case 8:  params.slowZoneDeg = value; break;
            case 9:  params.approachZoneDeg = value; break;
            case 10: params.limitInfDeg = value; break;
            case 11: params.limitSupDeg = value; break;
        }

        if (posEnd == -1) break;
        start = posEnd + 1;
        paramIndex++;
    }

    jointBus.sendTuning(jointId, params);
}

// "reload[3]" — pide a la articulación que reenvíe sus parámetros actuales
// y reporta al PC los últimos conocidos.
void handleReloadCommand(const String& line) {
    int idxL = line.indexOf('[');
    int idxR = line.indexOf(']');
    if (idxL == -1 || idxR == -1 || idxR <= idxL + 1) return;
    int jointId = line.substring(idxL + 1, idxR).toInt();
    if (jointId < 1 || jointId > 6) return;

    jointBus.sendReload(jointId);
    delay(50);
    Serial.println(jointBus.formatTuningReport(jointId));
    delay(100);  // pequeña pausa para asegurar que el mensaje se envíe completo
}

// "bomba=1" / "bomba=0"
void handleVacuumPumpCommand(const String& line) {
    int value = line.substring(6).toInt();
    digitalWrite(centralConfig::pinVacuumPump, value ? HIGH : LOW);
    Serial.printf("Bomba de vacío: %s\n", value ? "ON" : "OFF");
}

// "cinta=1;vel=75" -> arranca al 75%; "cinta=1" -> a la última velocidad
// conocida; "cinta=0" -> para.
void handleConveyorCommand(const String& line) {
    int start = 0;
    bool first = true;
    while (start < (int)line.length()) {
        int posEnd = line.indexOf(';', start);
        String token = (posEnd == -1) ? line.substring(start) : line.substring(start, posEnd);
        token.trim();

        if (first) {
            if (token.startsWith("cinta=")) {
                conveyorBus.setRun(token.substring(6).toInt() != 0);
            }
            first = false;
        } else if (token.startsWith("vel=")) {
            conveyorBus.setVelocityPercent(token.substring(4).toFloat());
        }

        if (posEnd == -1) break;
        start = posEnd + 1;
    }

    conveyorBus.sendCommand();
    Serial.printf("Cinta: %s (vel=%.0f%%)\n", conveyorBus.run() ? "ARRANQUE" : "PARO", conveyorBus.velocityPercent());
}

// "cintadist=4.5"
void handleConveyorDistanceCommand(const String& line) {
    float value = line.substring(10).toFloat();
    if (value <= 0.0f) {
        Serial.println("ERROR: distancia de deteccion de la cinta invalida (debe ser > 0)");
        return;
    }
    conveyorBus.setDetectionThresholdCm(value);
    conveyorBus.sendCommand();
    Serial.printf("Cinta: distancia de deteccion = %.1f cm\n", value);
}

void dispatchSerialCommand(const String& cmd) {
    if (cmd.equalsIgnoreCase("OTA")) {
        ota.enterOtaMode();
    } else if (cmd.startsWith("SP[")) {
        handleSetpointCommand(cmd);
        if (!emergencyStop.isActive()) jointBus.flushPendingSetpoints(pcVelocityPercent);
    } else if (cmd.startsWith("reset[")) {
        handleResetCommand(cmd);
    } else if (cmd.startsWith("enable[")) {
        if (!emergencyStop.isActive()) handleEnableCommand(cmd);  // ignorar durante emergencia
    } else if (cmd.startsWith("init[")) {
        handleInitCommand(cmd);
    } else if (cmd.startsWith("reload[")) {
        handleReloadCommand(cmd);
    } else if (cmd.startsWith("bomba=")) {
        handleVacuumPumpCommand(cmd);
    } else if (cmd.startsWith("cintadist=")) {
        // Se comprueba ANTES que "cinta=" porque ambos empiezan por "cinta".
        handleConveyorDistanceCommand(cmd);
    } else if (cmd.startsWith("cinta=")) {
        handleConveyorCommand(cmd);
    }
}

//------------------------------------------------------------------------
//  SETUP
//------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);  // PC

    pinMode(centralConfig::muxRxS0, OUTPUT);
    pinMode(centralConfig::muxRxS1, OUTPUT);
    pinMode(centralConfig::muxRxS2, OUTPUT);
    pinMode(centralConfig::muxTxS0, OUTPUT);
    pinMode(centralConfig::muxTxS1, OUTPUT);
    pinMode(centralConfig::muxTxS2, OUTPUT);

    beacon.begin(centralConfig::pinRed, centralConfig::pinGreen, centralConfig::pinYellow, centralConfig::pinHorn);

    pinMode(centralConfig::pinVacuumPump, OUTPUT);
    delay(10);
    digitalWrite(centralConfig::pinVacuumPump, LOW);

    delay(1000);

    // Ventana OTA de arranque: si en los primeros 30s alguien se conecta al
    // WiFi propio del central, entra en modo actualización y el resto de
    // setup() (WiFi_STA + ESP-NOW) se salta — loop() atenderá el servidor
    // OTA de forma no bloqueante.
    ota.bootWindowCheck();
    if (ota.isActive()) {
        Serial.println(F("Arranque en modo OTA: ESP-NOW no se inicializa"));
        return;
    }

    // Secuencia de arranque visible en la baliza.
    bool toggle = false;
    for (int i = 0; i <= 6; i++) {
        toggle = !toggle;
        beacon.setColors(!toggle, !toggle, !toggle);
        delay(500);
    }
    beacon.beep(100, 2);

    WiFi.mode(WIFI_MODE_STA);
    Serial.print("MAC del Central: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        return;
    }

    jointBus.begin();
    esp_now_register_recv_cb(onEspNowDataRecv);
    mandoBus.begin();
    conveyorBus.begin();

    // Seta de emergencia: SIEMPRE registrar DESPUES de WiFi/ESP-NOW, que
    // pueden resetear el controlador GPIO y borrar un attachInterrupt previo.
    emergencyStop.begin(centralConfig::pinEmergencyStop);

    beacon.update(false, false);  // estado inicial correcto (verde)

    Serial.println("Multiplexor 6DOF listo");
}

//------------------------------------------------------------------------
//  LOOP
//------------------------------------------------------------------------
void loop() {
    emergencyStop.update();

    if (emergencyStop.consumeChangedFlag()) {
        if (emergencyStop.isActive()) {
            jointBus.setAllDisabled(true);
            beacon.beep(150, 3);
            Serial.println("EMERGENCIA ACTIVADA - motores deshabilitados");
        } else {
            jointBus.setAllDisabled(false);
            beacon.beep(100, 1);
            Serial.println("EMERGENCIA LIBERADA - motores habilitados");
        }
    }

    // Modo OTA activo: atender el servidor web y saltarse el resto de
    // comandos (sin ESP-NOW no hay a quién reenviarlos). La seta de
    // emergencia se sigue comprobando siempre, esté o no en modo OTA.
    if (ota.isActive()) {
        ota.poll();
        return;
    }

    // 1) Comandos desde el PC
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) dispatchSerialCommand(cmd);
    }

    // 2) Setpoints pendientes que llegaron del mando por ESP-NOW
    {
        bool anyMandoSetpoint = false;
        for (int i = 0; i < 6; i++) {
            if (mandoSetpointPending[i]) {
                mandoSetpointPending[i] = false;
                anyMandoSetpoint = true;
                jointBus.setTargetAngle(i + 1, mandoSetpointValue[i]);
                pcVelocityPercent = constrain(mandoVelocityPercent[i], 5.0f, 100.0f);
                // Aviso al PC en formato parseable, para que la app sincronice
                // su cinemática con el setpoint que acaba de mandar el mando.
                Serial.printf("spmando[%d]=%.2f;\n", i + 1, mandoSetpointValue[i]);
            }
        }
        if (anyMandoSetpoint && !emergencyStop.isActive()) jointBus.flushPendingSetpoints(pcVelocityPercent);
    }

    // 3) Reset pendiente pedido desde el mando
    if (!emergencyStop.isActive()) {
        for (int i = 0; i < 6; i++) {
            if (mandoResetPending[i]) {
                mandoResetPending[i] = false;
                jointBus.sendReset(i + 1, /*preserveEnabled=*/true);
                Serial.printf("resetmando[%d]=1;\n", i + 1);
            }
        }
    }

    // 4) Reporte periódico al PC: ángulos de las 6 articulaciones + estado
    //    de la cinta transportadora.
    unsigned long now = millis();
    if (now - lastAngleReportMs >= ANGLE_REPORT_INTERVAL_MS) {
        lastAngleReportMs = now;
        String report = "";
        for (int i = 1; i <= 6; i++) {
            report += "angulo[" + String(i) + "]=" + String(jointBus.angleDeg(i)) + ";";
        }
        report += "objetoCinta=" + String(conveyorBus.objectDetected() ? 1 : 0) + ";";
        report += "distCinta=" + String(conveyorBus.lastDistanceCm(), 1) + ";";
        Serial.println(report);
    }

    // 5) Reporte periódico de ángulos al Mando
    if (now - lastMandoReportMs >= MANDO_REPORT_MS) {
        lastMandoReportMs = now;
        float angles[6];
        for (int i = 0; i < 6; i++) angles[i] = jointBus.angleDeg(i + 1);
        mandoBus.reportAngles(angles);
    }

    // 6) Baliza según estado del sistema
    beacon.update(emergencyStop.isActive(), jointBus.isAnyJointMoving());
}
