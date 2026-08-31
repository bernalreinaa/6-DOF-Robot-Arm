//------------------------------------------------------------------------
//  Control de posición angular de una articulación (motor NEMA17 + encoder
//  AS5600 + driver TB6600) sobre ESP32-C3, comunicada por ESP-NOW con el
//  Central central.
//
//  Este fichero es el ÚNICO punto de orquestación: crea los objetos de las
//  librerías (lib/), registra las tareas FreeRTOS y conecta los callbacks
//  de ESP-NOW con JointMotionController. TODA la lógica de control vive en
//  las librerías — ver lib/JointMotionController, lib/AngleMath,
//  lib/MagneticEncoder, lib/StepperDriver, lib/JointDisplay,
//  lib/JointStorage, lib/JointOta y lib/EspNowProtocol.
//
//  Este mismo main.cpp es IDÉNTICO en las 6 articulaciones: lo único que
//  cambia entre ellas es lib/JointConfig/JointConfig.h.
//
//  Conexiones (iguales en las 6 placas):
//    - AS5600: VCC a 3.3V, GND a GND, SDA a GPIO5, SCL a GPIO6
//    - TB6600: PUL+/DIR+ a los pines del ESP32-C3, PUL-/DIR-/EN- a GND
//    - OLED SH1106: VCC a 3.3V, GND a GND, SDA a GPIO5, SCL a GPIO6
//------------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "JointConfig.h"
#include "EspNowProtocol.h"
#include "AngleMath.h"
#include "MagneticEncoder.h"
#include "StepperDriver.h"
#include "JointDisplay.h"
#include "JointMotionController.h"
#include "JointStorage.h"
#include "JointOta.h"

//------------------------------------------------------------------------
//  Objetos de las librerías
//------------------------------------------------------------------------
MagneticEncoder encoder;
StepperDriver stepper;
JointDisplay display;
JointStorage storage;
JointOta ota(display, jointConfig::id);

JointMechanics mechanics{
    jointConfig::ratio,
    jointConfig::motorStepsPerRev,
    jointConfig::driverMicrosteps,
    jointConfig::encoderOnOutputSide,
    jointConfig::encoderGearRatio,
    jointConfig::invertDir,
    jointConfig::backlashOutDeg,
    kDefaultPulseHighUs,
    kDefaultPulseLowUs,
    kDefaultDirSetupUs,
    jointConfig::integralMax,
    kDefaultControlPeriodMs,
};

JointTuning tuning{
    jointConfig::kp, jointConfig::ki, jointConfig::kd,
    jointConfig::maxVelDegS, jointConfig::cruiseVelDegS,
    jointConfig::approachVelDegS, jointConfig::minVelDegS,
    jointConfig::tolDeg, jointConfig::slowZoneDeg, jointConfig::approachZoneDeg,
    jointConfig::limitInfDeg, jointConfig::limitSupDeg,
};

JointMotionController motion(mechanics, tuning, stepper);

//------------------------------------------------------------------------
//  Estado compartido entre tareas / callbacks ESP-NOW
//------------------------------------------------------------------------
TaskHandle_t motionTaskHandle = nullptr;

AngleMessage    outgoingAngle{jointConfig::id, 0.0f};
TuningParams    outgoingTuning{};
SetpointCommand incomingSetpoint{};
TuningParams    incomingTuning{};
ReloadCommand   incomingReload{};

volatile bool  lastSendOk = true;
volatile bool  resetRequested = false;
volatile bool  motorDisabled = false;   // ver EspNowProtocol::SetpointCommand
volatile float velocityScale = 1.0f;
volatile bool  paramsDirty = false;     // guardar en flash pendiente (se hace desde loop())

void fillOutgoingTuning() {
    outgoingTuning.id              = jointConfig::id;
    outgoingTuning.kp              = motion.tuning().kp;
    outgoingTuning.ki              = motion.tuning().ki;
    outgoingTuning.kd              = motion.tuning().kd;
    outgoingTuning.maxVelDegS      = motion.tuning().maxVelDegS;
    outgoingTuning.cruiseVelDegS   = motion.tuning().cruiseVelDegS;
    outgoingTuning.approachVelDegS = motion.tuning().approachVelDegS;
    outgoingTuning.minVelDegS      = motion.tuning().minVelDegS;
    outgoingTuning.tolDeg          = motion.tuning().tolDeg;
    outgoingTuning.slowZoneDeg     = motion.tuning().slowZoneDeg;
    outgoingTuning.approachZoneDeg = motion.tuning().approachZoneDeg;
    outgoingTuning.limitInfDeg     = motion.tuning().limitInfDeg;
    outgoingTuning.limitSupDeg     = motion.tuning().limitSupDeg;
}

//------------------------------------------------------------------------
//  ESP-NOW: callbacks
//------------------------------------------------------------------------
void onEspNowDataSent(const uint8_t* /*macAddr*/, esp_now_send_status_t status) {
    lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

// NOTA: por convención de Espressif, este callback NO debe llamar a
// esp_now_send() ni hacer operaciones lentas (flash, etc.) directamente:
// solo actualiza variables/flags; loop() y las tareas son quienes actúan.
void onEspNowDataRecv(const uint8_t* /*mac*/, const uint8_t* incomingData, int len) {
    if (len == sizeof(ReloadCommand)) {
        memcpy(&incomingReload, incomingData, sizeof(incomingReload));
        if (incomingReload.reload) {
            Serial.println("Reload recibido: reenviando parametros actuales");
            fillOutgoingTuning();
            esp_now_send(jointConfig::centralMac, (uint8_t*)&outgoingTuning, sizeof(outgoingTuning));
        }
    } else if (len == sizeof(SetpointCommand)) {
        memcpy(&incomingSetpoint, incomingData, sizeof(incomingSetpoint));

        if (incomingSetpoint.id != jointConfig::id) return;

        motorDisabled = incomingSetpoint.motorDisabled;
        velocityScale = constrain(incomingSetpoint.velocityPercent / 100.0f, 0.05f, 1.0f);

        Serial.printf("ID: %d | SP: %.2f | RST: %s | DIS: %s | VEL: %.0f%%\n",
                      incomingSetpoint.id, incomingSetpoint.setpointDeg,
                      incomingSetpoint.reset ? "ON" : "OFF",
                      incomingSetpoint.motorDisabled ? "ON" : "OFF",
                      incomingSetpoint.velocityPercent);

        if (incomingSetpoint.reset) {
            resetRequested = true;
        } else if (!motion.isInForbiddenZone(incomingSetpoint.setpointDeg)) {
            // El valor en si queda en incomingSetpoint.setpointDeg; la
            // notificacion solo avisa a motionTask de que hay uno nuevo que
            // consumir (igual que el firmware original).
            if (motionTaskHandle) xTaskNotifyGive(motionTaskHandle);
        } else {
            Serial.printf("AVISO: setpoint %.2f grados esta en zona prohibida, ignorado\n",
                          incomingSetpoint.setpointDeg);
        }
    } else if (len == sizeof(TuningParams)) {
        memcpy(&incomingTuning, incomingData, sizeof(incomingTuning));
        paramsDirty = true;  // guardar en flash desde loop(), nunca desde aqui

        JointTuning& t = motion.tuning();
        if (incomingTuning.kp != t.kp) t.kp = incomingTuning.kp;
        if (incomingTuning.ki != t.ki) t.ki = incomingTuning.ki;
        if (incomingTuning.kd != t.kd) t.kd = incomingTuning.kd;
        if (incomingTuning.maxVelDegS != t.maxVelDegS) t.maxVelDegS = incomingTuning.maxVelDegS;
        if (incomingTuning.cruiseVelDegS != t.cruiseVelDegS) t.cruiseVelDegS = incomingTuning.cruiseVelDegS;
        if (incomingTuning.approachVelDegS != t.approachVelDegS) t.approachVelDegS = incomingTuning.approachVelDegS;
        if (incomingTuning.minVelDegS != t.minVelDegS) t.minVelDegS = incomingTuning.minVelDegS;
        if (incomingTuning.tolDeg != t.tolDeg) t.tolDeg = incomingTuning.tolDeg;
        if (incomingTuning.slowZoneDeg != t.slowZoneDeg) t.slowZoneDeg = incomingTuning.slowZoneDeg;
        if (incomingTuning.approachZoneDeg != t.approachZoneDeg) t.approachZoneDeg = incomingTuning.approachZoneDeg;
        if (incomingTuning.limitInfDeg != t.limitInfDeg) t.limitInfDeg = incomingTuning.limitInfDeg;
        if (incomingTuning.limitSupDeg != t.limitSupDeg) t.limitSupDeg = incomingTuning.limitSupDeg;
    }
}

//------------------------------------------------------------------------
//  TAREA 1 — ENCODER (lectura de posición + reporte por ESP-NOW + OLED)
//------------------------------------------------------------------------
void encoderTask(void* /*param*/) {
    for (;;) {
        double rawAccumDeg = encoder.pollAccumulatedDegrees();
        double outputAngleDeg = motion.updateFeedback(rawAccumDeg);

        Serial.printf("angulo:%.3f \r\n", outputAngleDeg);

        double errorDeg = angleMath::wrapTo180(incomingSetpoint.setpointDeg - outputAngleDeg);

        display.showStatus((float)incomingSetpoint.setpointDeg, (float)fabs(errorDeg),
                            (float)outputAngleDeg, lastSendOk);

        outgoingAngle.id = jointConfig::id;
        outgoingAngle.angleDeg = (float)outputAngleDeg;
        esp_now_send(jointConfig::centralMac, (uint8_t*)&outgoingAngle, sizeof(outgoingAngle));

        stepper.setDisabled(motorDisabled);
        if (motorDisabled) {
            Serial.printf("Articulacion:%d deshabilitada \r\n", jointConfig::id);
        }

        if (resetRequested) {
            motion.requestReset();
            resetRequested = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//------------------------------------------------------------------------
//  TAREA 2 — MOVIMIENTO (PID + generación de pulsos NEMA17)
//------------------------------------------------------------------------
void motionTask(void* /*param*/) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // espera nueva orden (setpoint)

        motion.beginMove(incomingSetpoint.setpointDeg);
        Serial.printf("Iniciando movimiento -> SP=%.2f Act=%.2f\r\n",
                      incomingSetpoint.setpointDeg, motion.outputAngleDeg());

        while (true) {
            if (ulTaskNotifyTake(pdTRUE, 0) == pdTRUE) {
                motion.beginMove(incomingSetpoint.setpointDeg);
                Serial.printf("Actualizando movimiento -> SP=%.2f\r\n", incomingSetpoint.setpointDeg);
            }

            bool reached = motion.stepOnce(velocityScale);
            if (reached) {
                Serial.println("Posicion alcanzada");
                break;
            }

            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS((uint32_t)mechanics.controlPeriodMs));
        }

        Serial.println("Movimiento completado");
    }
}

//------------------------------------------------------------------------
//  SETUP
//------------------------------------------------------------------------
void setup() {
    delay(3000);
    Serial.begin(115200);

    display.begin();

    // Ventana OTA de arranque: si en los primeros 30s alguien se conecta al
    // WiFi propio de esta articulación, se queda en modo actualización y
    // nunca vuelve de aquí. Si nadie se conecta, sigue el arranque normal.
    ota.bootWindowCheck();

    // cargarParametrosDesdeFlash() (acceso a NVS) va DESPUES de la ventana
    // OTA: la primera vez que el NVS se toca tras reflashear puede tardar
    // varios segundos, lo que retrasaría la ventana de 30s.
    storage.load(motion.tuning());

    encoder.begin(jointConfig::i2cSdaPin, jointConfig::i2cSclPin);

    WiFi.mode(WIFI_STA);
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        return;
    }

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, jointConfig::centralMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error al anadir el central como peer");
        return;
    }

    // El cast explicito evita un mismatch de firma entre versiones del core
    // arduino-esp32 (algunas exigen const uint8_t* en el primer parametro,
    // otras uint8_t*); se mantiene por compatibilidad, igual que en el
    // firmware original.
    esp_now_register_recv_cb(esp_now_recv_cb_t(onEspNowDataRecv));
    esp_now_register_send_cb(onEspNowDataSent);

    display.showThreeLines("", "Iniciando", "");

    delay(1000);
    Serial.println(encoder.isConnected() ? " AS5600 Conectado" : " AS5600 No Conectado");
    delay(1000);

    encoder.setDirectionCounterClockwise();

    stepper.begin(jointConfig::pinPul, jointConfig::pinDir, jointConfig::pinEnable);

    Serial.println("Listo. Esperando setpoints por ESP-NOW.");

    fillOutgoingTuning();
    delay(100);
    esp_now_send(jointConfig::centralMac, (uint8_t*)&outgoingTuning, sizeof(outgoingTuning));
    delay(1000);

    // Tareas en el nucleo 0 (el unico del ESP32-C3)
    xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motionTask, "MotionTask", 4096, NULL, 1, &motionTaskHandle, 0);
}

//------------------------------------------------------------------------
//  LOOP — persistencia en flash pendiente + comando OTA manual por Serial
//------------------------------------------------------------------------
void loop() {
    if (paramsDirty) {
        paramsDirty = false;
        storage.save(motion.tuning());
    }

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("OTA")) {
            ota.enterOtaMode();
        }
    }
    delay(5);
}
