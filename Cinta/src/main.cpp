//------------------------------------------------------------------------
//  Cinta transportadora (ESP32-C3) — motor NEMA17 + driver TB6600 en lazo
//  ABIERTO (sin encoder: se controla velocidad, no una posición angular
//  absoluta como en las articulaciones del brazo), sensor HC-SR04 para
//  detección de piezas y parada de seguridad, pantalla OLED SSD1306 de
//  0.42", y puente ESP-NOW con el Central.
//
//  Este fichero es el punto de orquestación: crea los objetos de las
//  librerías (lib/) y las 4 tareas FreeRTOS. TODA la lógica de cada
//  subsistema vive en su librería — ver lib/ConveyorDrive,
//  lib/UltrasonicSensor, lib/ConveyorDisplay y lib/ConveyorProtocol.
//
//  Comandos por Serial (Monitor Serie, 115200 baudios, terminador NL o CR+NL):
//    V<valor>   Velocidad objetivo en % de la velocidad máxima, con signo.
//               Rango -100..100. Positivo = adelante, negativo = atrás.
//    S / STOP   Para la cinta (equivalente a "V0").
//    D          Muestra la última distancia medida por el HC-SR04.
//    ? / HELP   Muestra esta ayuda por Serial.
//
//  Conexiones:
//    - TB6600: PUL+/DIR+ a los pines del ESP32-C3, PUL-/DIR-/EN- a GND
//    - OLED SSD1306 0.42" (72x40): VCC a 3.3V, GND a GND, SDA/SCL compartidos
//    - HC-SR04: TRIG/ECHO según ConveyorConfig (¡ECHO no tolera 5V directo!)
//------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#include "ConveyorConfig.h"
#include "ConveyorProtocol.h"
#include "ConveyorDrive.h"
#include "UltrasonicSensor.h"
#include "ConveyorDisplay.h"

//------------------------------------------------------------------------
//  Objetos de las librerías
//------------------------------------------------------------------------
ConveyorDrive drive;
UltrasonicSensor sensor;
ConveyorDisplay display;

//------------------------------------------------------------------------
//  Estado compartido entre tareas (fotos simples de 32 bits: un único
//  escritor por variable, no hace falta mutex — ver comentarios originales)
//------------------------------------------------------------------------
volatile float targetVelocityPercent = 0.0f;   // fijada por Serial o por el Central (ESP-NOW)
volatile float lastDistanceCm = -1.0f;         // escrita solo por sensorTask
volatile bool  obstacleDetected = false;       // escrita solo por sensorTask
volatile float obstacleThresholdCm = conveyorConfig::obstacleDistanceDefaultCm;

//------------------------------------------------------------------------
//  ESP-NOW con el Central
//------------------------------------------------------------------------
void sendConveyorStatus() {
    ConveyorStatus status{};
    status.marker = 0;
    status.objectDetected = obstacleDetected;
    status.distanceCm = lastDistanceCm;
    esp_now_send(conveyorConfig::centralMac, (uint8_t*)&status, sizeof(status));
}

// Recibe el comando de la app (vía central) y lo aplica de inmediato.
void onEspNowDataRecv(const uint8_t* /*mac*/, const uint8_t* incomingData, int len) {
    if (len != sizeof(ConveyorCommand)) return;  // paquete de otro tamaño/tipo: no es para nosotros

    ConveyorCommand cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));

    targetVelocityPercent = cmd.run ? constrain(cmd.velocityPercent, 0.0f, 100.0f) : 0.0f;
    if (cmd.detectionThresholdCm > 0.0f) {
        obstacleThresholdCm = cmd.detectionThresholdCm;
    }
}

//------------------------------------------------------------------------
//  TAREA 1 — SUPERVISOR DE VELOCIDAD
//------------------------------------------------------------------------
// NO genera los pulsos (eso lo hace ConveyorDrive vía esp_timer). Solo
// vigila targetVelocityPercent/obstacleDetected y reconfigura el driver
// cuando cambia algo relevante. Cede CPU en cada vuelta con vTaskDelay().
void conveyorTask(void* /*param*/) {
    float lastApplied = NAN;  // fuerza aplicar la velocidad inicial (0) la primera vez

    for (;;) {
        // Si hay una pieza a <= obstacleThresholdCm del HC-SR04 se fuerza
        // velocidad 0 aunque targetVelocityPercent siga pidiendo movimiento.
        // En cuanto la pieza se aleja, la cinta retoma sola la velocidad
        // pedida, sin reenviar el comando.
        float velocityPercent = obstacleDetected ? 0.0f : targetVelocityPercent;

        bool relevantChange = isnan(lastApplied)
            || (fabs(velocityPercent - lastApplied) > 0.4f)
            || ((fabs(velocityPercent) < 0.5f) != (fabs(lastApplied) < 0.5f));

        if (relevantChange) {
            drive.setVelocityPercent(velocityPercent);
            lastApplied = velocityPercent;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

//------------------------------------------------------------------------
//  TAREA 2 — COMANDOS POR SERIAL
//------------------------------------------------------------------------
void printHelp() {
    Serial.println(F("--------------------------------------------"));
    Serial.println(F(" CINTA TRANSPORTADORA -- comandos Serial"));
    Serial.println(F("  V<valor>   Velocidad -100..100 (%)"));
    Serial.println(F("             Ej: V50  V-30  V0"));
    Serial.println(F("  S / STOP   Parar la cinta"));
    Serial.println(F("  D          Distancia HC-SR04 (cm)"));
    Serial.println(F("  ? / HELP   Esta ayuda"));
    Serial.println(F("--------------------------------------------"));
}

// Valida y aplica un comando ya recibido completo (sin el salto de línea).
// Cualquier entrada que no encaje con los comandos soportados se rechaza
// con un mensaje claro, en vez de interpretarse silenciosamente como 0.
void processSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

    if (cmdUpper == "S" || cmdUpper == "STOP") {
        targetVelocityPercent = 0.0f;
        Serial.println(F("CINTA PARADA"));
        return;
    }

    if (cmdUpper == "?" || cmdUpper == "HELP") {
        printHelp();
        return;
    }

    if (cmdUpper == "D") {
        float d = lastDistanceCm;
        if (d < 0.0f) {
            Serial.println(F("Distancia: sin eco / fuera de rango"));
        } else {
            Serial.printf("Distancia: %.1f cm\r\n", d);
        }
        return;
    }

    if (cmdUpper.charAt(0) == 'V' && cmdUpper.length() >= 2) {
        String numStr = cmdUpper.substring(1);
        char* endptr = nullptr;
        float val = strtod(numStr.c_str(), &endptr);

        bool validNumber = (endptr != numStr.c_str()) && (*endptr == '\0');
        if (!validNumber) {
            Serial.printf("Comando invalido: '%s' (esperado V<-100..100>)\r\n", cmd.c_str());
            return;
        }

        if (val < -100.0f || val > 100.0f) {
            Serial.printf("Velocidad %.1f fuera de rango, se recorta a [-100,100]\r\n", val);
        }
        val = constrain(val, -100.0f, 100.0f);
        targetVelocityPercent = val;

        const char* sentido = (fabs(val) < 0.5f) ? "PARADO" : (val > 0 ? "ADELANTE" : "ATRAS");
        Serial.printf("Velocidad objetivo: %.1f %% (%s)\r\n", val, sentido);
        return;
    }

    Serial.printf("Comando no reconocido: '%s'. Escribe '?' para ayuda.\r\n", cmd.c_str());
}

void serialTask(void* /*param*/) {
    String buffer = "";
    const size_t kMaxCmdLen = 32;  // protección contra basura/ruido en el puerto

    for (;;) {
        while (Serial.available() > 0) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                if (buffer.length() > 0) {
                    processSerialCommand(buffer);
                    buffer = "";
                }
            } else {
                buffer += c;
                if (buffer.length() > kMaxCmdLen) {
                    Serial.println(F("Comando demasiado largo, descartado"));
                    buffer = "";
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//------------------------------------------------------------------------
//  TAREA 3 — SENSOR ULTRASÓNICO HC-SR04
//------------------------------------------------------------------------
// Mide la distancia periódicamente (unas 8 veces/segundo, de sobra para ver
// pasar una pieza por la cinta). Al estar en su propia tarea, a la misma
// prioridad que el resto, el planificador sigue repartiendo CPU aunque esta
// tarea esté "ocupada" esperando el eco.
void sensorTask(void* /*param*/) {
    bool obstaclePrev = false;  // para avisar por Serial solo al cambiar de estado

    for (;;) {
        float distanceCm = sensor.measureCm();
        lastDistanceCm = distanceCm;

        // Sin lectura válida no se puede afirmar que haya pieza: no se para
        // la cinta en ese caso.
        bool obstacleNow = (distanceCm >= 0.0f) && (distanceCm <= obstacleThresholdCm);
        obstacleDetected = obstacleNow;

        if (obstacleNow != obstaclePrev) {
            if (obstacleNow) {
                Serial.printf("Pieza detectada a <=%.1fcm: cinta detenida por seguridad\r\n",
                              (float)obstacleThresholdCm);
            } else {
                Serial.println(F("Via libre: la cinta retoma la velocidad pedida"));
            }
            obstaclePrev = obstacleNow;
        }

        // Reporta el estado del sensor al central por ESP-NOW en cada
        // ciclo (~8 Hz) — la app lo usa para la condición "pieza
        // detectada / no detectada" en las rutinas.
        sendConveyorStatus();

        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

//------------------------------------------------------------------------
//  TAREA 4 — PANTALLA OLED
//------------------------------------------------------------------------
// Tarea SEPARADA de conveyorTask() a propósito: las llamadas I2C de
// sendBuffer() tardan del orden de varios ms, y si vivieran en la misma
// tarea que genera los pulsos del motor introducirían huecos irregulares
// en el tren de pulsos.
void oledTask(void* /*param*/) {
    for (;;) {
        float velocityPercent = targetVelocityPercent;
        float distanceCm = lastDistanceCm;
        bool obstacle = obstacleDetected;

        // p/s real aplicado al motor: 0 si está parada por obstáculo,
        // aunque targetVelocityPercent siga pidiendo movimiento.
        float stepsPerSecond = drive.currentStepsPerSecond();

        display.showStatus(velocityPercent, distanceCm, obstacle, stepsPerSecond);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

//------------------------------------------------------------------------
//  SETUP
//------------------------------------------------------------------------
void setup() {
    delay(500);  // breve espera para estabilizar la alimentación al arrancar
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    Serial.print(F("MAC ESP-NOW: "));
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println(F("Error inicializando ESP-NOW"));
    } else {
        Serial.println(F("ESP-NOW inicializado"));

        esp_now_peer_info_t peerCentral = {};
        memcpy(peerCentral.peer_addr, conveyorConfig::centralMac, 6);
        peerCentral.channel = 0;
        peerCentral.encrypt = false;
        if (esp_now_add_peer(&peerCentral) == ESP_OK) {
            Serial.println(F("Central registrado como peer ESP-NOW"));
        } else {
            Serial.println(F("Error registrando al central como peer"));
        }

        esp_now_register_recv_cb(esp_now_recv_cb_t(onEspNowDataRecv));
    }

    // Desactiva el ahorro de energía del WiFi: el modem sleep mete picos de
    // latencia que empeoran la medición del HC-SR04 (ver UltrasonicSensor);
    // con ESP-NOW no hace falta ahorrar energía de WiFi de todas formas.
    WiFi.setSleep(false);

    Wire.begin(conveyorConfig::i2cSdaPin, conveyorConfig::i2cSclPin, 100000);
    display.begin();
    display.showMessage("Iniciando");

    pinMode(conveyorConfig::pinEnable, OUTPUT);
    digitalWrite(conveyorConfig::pinEnable, HIGH);  // mismo criterio que en las articulaciones (EN no cableado)

    float maxSpeedStepsPerSecond =
        (conveyorConfig::maxSpeedRpm / 60.0f)
        * (conveyorConfig::motorStepsPerRev * conveyorConfig::driverMicrosteps)
        * conveyorConfig::reduction;

    drive.begin(conveyorConfig::pinPul, conveyorConfig::pinDir,
                conveyorConfig::pulseHighUs, conveyorConfig::dirSetupUs,
                maxSpeedStepsPerSecond, conveyorConfig::invertDir);

    sensor.begin(conveyorConfig::pinTrig, conveyorConfig::pinEcho,
                 conveyorConfig::echoTimeoutUs, conveyorConfig::medianSamples);

    Serial.println(F("Cinta transportadora lista."));
    printHelp();

    // Tareas en el único núcleo del ESP32-C3. Ninguna genera los pulsos del
    // motor directamente (eso lo hace el esp_timer de ConveyorDrive,
    // independiente del scheduler), así que las cuatro pueden ir a la misma
    // prioridad sin que ninguna se quede sin CPU.
    xTaskCreatePinnedToCore(conveyorTask, "ConveyorTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(serialTask,   "SerialTask",   4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(oledTask,     "OledTask",     4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask,   "SensorTask",   4096, NULL, 1, NULL, 0);
}

//------------------------------------------------------------------------
//  LOOP — no se usa: toda la lógica corre en tareas FreeRTOS.
//------------------------------------------------------------------------
void loop() {}
