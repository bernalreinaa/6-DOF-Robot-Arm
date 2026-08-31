//------------------------------------------------------------------------
//  Mando físico (ESP32-S3) — pantalla táctil Nextion + rueda encoder +
//  pulsadores + seta de emergencia (leídos por un microcontrolador
//  secundario y reportados por UART, ver lib/NanoLink) + puente ESP-NOW con
//  el Central central.
//
//  Este fichero es el punto de orquestación: crea los objetos de las
//  librerías (lib/) y las 2 tareas FreeRTOS (touchTask + nextionTask). TODA
//  la lógica de cada subsistema vive en su librería — ver lib/NextionUi,
//  lib/NanoLink, lib/CentralLink, lib/StepSequence, lib/MandoOta y
//  lib/EspNowProtocol.
//------------------------------------------------------------------------

#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>
#include <math.h>

#include "MandoConfig.h"
#include "NextionUi.h"
#include "NanoLink.h"
#include "CentralLink.h"
#include "StepSequence.h"
#include "MandoOta.h"

//------------------------------------------------------------------------
//  Objetos de las librerías
//------------------------------------------------------------------------
HardwareSerial nextionSerial(2);
HardwareSerial auxSerial(1);

NextionUi nextion;
NanoLink nano;
CentralLink central;
StepSequence stepSequence;
MandoOta ota;

//------------------------------------------------------------------------
//  Estado de la interfaz
//------------------------------------------------------------------------
float angulosEnviados[6]  = {-999, -999, -999, -999, -999, -999};  // último valor enviado a Nextion, para no reenviar sin cambios
float pos      = 0.0f;
int   pantalla = 0;

// Último SETPOINT marcado (x0) por articulación — no es el ángulo real, es
// el valor que se estaba marcando con la rueda aunque no se haya confirmado
// con el pulsador. -1 = "todavía no se ha marcado nada en esta articulación".
float setpointsPorArticulacion[6] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

// Velocidades por articulación (%), enviadas al Central junto con cada setpoint.
float velocidades[6] = {mandoConfig::velocityDefault, mandoConfig::velocityDefault,
                         mandoConfig::velocityDefault, mandoConfig::velocityDefault,
                         mandoConfig::velocityDefault, mandoConfig::velocityDefault};

unsigned long lastDisplayUpdate = 0;
volatile bool escalonCambio = false;  // sincroniza refresco de x20 tras cambiar el paso desde otra tarea

// Último setpoint realmente enviado por articulación, para filtrar
// duplicados (p.ej. ruido del encoder repitiendo el mismo valor).
float ultimoSetpointEnviado[6] = {-9999, -9999, -9999, -9999, -9999, -9999};
unsigned long ultimoEnvioMs = 0;

//------------------------------------------------------------------------
//  Envío de setpoint al Central (con filtro de duplicados + throttle)
//------------------------------------------------------------------------
void enviarSetpointCentral(int articulacion, float angulo) {
    if (articulacion >= 1 && articulacion <= 6) {
        if (fabs(angulo - ultimoSetpointEnviado[articulacion - 1]) < 0.01f) return;
        ultimoSetpointEnviado[articulacion - 1] = angulo;
    }

    unsigned long ahora = millis();
    if (ahora - ultimoEnvioMs < 20) return;
    ultimoEnvioMs = ahora;

    float velocidad = (articulacion >= 1 && articulacion <= 6) ? velocidades[articulacion - 1] : 100.0f;
    central.sendSetpoint(articulacion, angulo, velocidad);
}

void irAHome() {
    for (int i = 1; i <= 6; i++) {
        setpointsPorArticulacion[i - 1] = 0.0f;
        enviarSetpointCentral(i, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(25));  // >20 ms para respetar el throttle interno
    }
    if (pantalla >= 1 && pantalla <= 6) {
        pos = 0.0f;
        nextion.sendFloat("x0", pos, 2);
    }
}

//------------------------------------------------------------------------
//  TAREA TOUCH (Core 0) — botones táctiles capacitivos
//------------------------------------------------------------------------
void invalidarCacheAngulos() {
    for (int i = 0; i < 6; i++) angulosEnviados[i] = -9999;
}

// Cambia a la página de una articulación (1-6): navega en la Nextion, y
// CARGA tanto el paso como el SETPOINT que se estaban usando en esa
// articulación. Si es la primera vez que se visita, arranca desde el
// ángulo real.
void seleccionarArticulacion(int n, const char* paginaNextion) {
    pantalla = n;
    nextion.changePage(paginaNextion);

    float setpointInicial = (setpointsPorArticulacion[n - 1] >= 0.0f)
                                 ? setpointsPorArticulacion[n - 1]
                                 : central.lastAngleDeg(n);
    pos = setpointInicial;
    setpointsPorArticulacion[n - 1] = setpointInicial;
    nano.send("PAN:" + String(n) + ":" + String(setpointInicial) + "\n");
    nextion.sendFloat("x0", setpointInicial, 2);  // refresco inmediato, sin esperar al enlace auxiliar

    stepSequence.recallForJoint(n);
    escalonCambio = true;  // refresca x20 en el siguiente ciclo de nextionTask
    nano.send("ESC:" + String((float)stepSequence.current()) + "\n");

    // Refresca el campo de velocidad de esta página (x11..x16), por si se
    // ajustó en una visita anterior o desde la app de Python.
    nextion.sendFloat("x1" + String(n), velocidades[n - 1], 0);

    invalidarCacheAngulos();
}

void touchTask(void* /*parameter*/) {
    for (;;) {
        if (touchRead(mandoConfig::touchPinArt1) > mandoConfig::touchThreshold) seleccionarArticulacion(1, "arti1");
        if (touchRead(mandoConfig::touchPinArt2) > mandoConfig::touchThreshold) seleccionarArticulacion(2, "arti2");
        if (touchRead(mandoConfig::touchPinArt3) > mandoConfig::touchThreshold) seleccionarArticulacion(3, "arti3");
        if (touchRead(mandoConfig::touchPinArt4) > mandoConfig::touchThreshold) seleccionarArticulacion(4, "arti4");
        if (touchRead(mandoConfig::touchPinArt5) > mandoConfig::touchThreshold) seleccionarArticulacion(5, "arti5");
        if (touchRead(mandoConfig::touchPinArt6) > mandoConfig::touchThreshold) seleccionarArticulacion(6, "arti6");
        if (touchRead(mandoConfig::touchPinInicio) > mandoConfig::touchThreshold) {
            pantalla = 7; nextion.changePage("inicio"); nano.send("PAN:7\n"); invalidarCacheAngulos();
        }
        if (touchRead(mandoConfig::touchPinAjustes) > mandoConfig::touchThreshold) {
            pantalla = 8; nextion.changePage("ajustes"); nano.send("PAN:8\n");
        }
        if (touchRead(mandoConfig::touchPinTareas) > mandoConfig::touchThreshold) {
            pantalla = 9; nextion.changePage("tareas"); nano.send("PAN:9\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//------------------------------------------------------------------------
//  TAREA NEXTION + ENLACE AUXILIAR (Core 0)
//------------------------------------------------------------------------
void nextionTask(void* /*parameter*/) {
    for (;;) {
        // Nextion -> Mando: botones +/- de cada pantalla ("art3:+5.0").
        // Suman/restan su valor al SETPOINT marcado (x0), igual que la
        // rueda: NO mueven el brazo todavía, hace falta confirmar con el
        // pulsador del encoder (BTN:OK).
        if (nextion.available()) {
            // La Nextion envía paquetes de evento táctil binarios antes del
            // print "artN:inc\n". Descartamos todo hasta encontrar 'a'.
            unsigned long tIni = millis();
            while (nextion.available() && (millis() - tIni) < 20) {
                char c = (char)nextion.read();
                if (c == 'a') {
                    nextion.setTimeout(10);
                    String resto = nextion.readStringUntil('\n');
                    String datos = String('a') + resto;
                    datos.trim();

                    if (datos.startsWith("artV:")) {
                        // Botones VELO.(%) — se usa "pantalla" para saber a
                        // qué articulación aplica (mismo texto en las 6 páginas).
                        float inc = datos.substring(5).toFloat();
                        if (pantalla >= 1 && pantalla <= 6) {
                            float nueva = constrain(velocidades[pantalla - 1] + inc,
                                                     mandoConfig::velocityMin, mandoConfig::velocityMax);
                            velocidades[pantalla - 1] = nueva;
                            nextion.sendFloat("x1" + String(pantalla), nueva, 0);
                        }
                    } else if (datos.startsWith("art")) {
                        int col = datos.indexOf(':');
                        if (col != -1) {
                            float inc = datos.substring(col + 1).toFloat();
                            // Se ignora deliberadamente el número de articulación
                            // que venga en "artN:valor" y se usa "pantalla" en su
                            // lugar (ver comentario histórico en el firmware original
                            // sobre páginas Nextion copiadas sin actualizar el texto).
                            if (pantalla >= 1 && pantalla <= 6) {
                                pos = fmod(pos + inc, 360.0f);
                                if (pos < 0.0f) pos += 360.0f;

                                setpointsPorArticulacion[pantalla - 1] = pos;
                                nextion.sendFloat("x0", pos, 2);
                                nano.send("SET:" + String(pos) + "\n");
                            }
                        }
                    }
                    break;
                }
            }
        }

        // Enlace auxiliar -> Mando: encoder (POS), seta (SETA), botón OK (BTN)
        if (nano.available()) {
            String msg = nano.readLine();
            int col = msg.indexOf(':');
            if (col != -1) {
                String etiqueta = msg.substring(0, col);
                String valor    = msg.substring(col + 1);

                if (etiqueta == "POS") {
                    // Girar la rueda solo actualiza el SETPOINT que se está
                    // marcando (x0). NO se toca el ángulo real ni se envía nada todavía.
                    pos = valor.toFloat();
                    if (pantalla >= 1 && pantalla <= 6) setpointsPorArticulacion[pantalla - 1] = pos;
                    nextion.sendFloat("x0", pos, 2);

                } else if (etiqueta == "BTN" && valor == "OK") {
                    // Pulsador del encoder: confirma y envía el setpoint
                    // marcado a la articulación activa para que se mueva.
                    if (pantalla >= 1 && pantalla <= 6) {
                        enviarSetpointCentral(pantalla, pos);
                    }

                } else if (etiqueta == "SETA") {
                    bool pulsada = (valor == "PULSADA");
                    Serial.println(pulsada ? "SETA PULSADA" : "SETA REARMADA");
                    central.sendEmergency(pulsada);

                } else if (etiqueta == "HOMEGO") {
                    // SUBIR+BAJAR a la vez: home real siempre, las 6 a 0°.
                    Serial.println("HOMEGO solicitado");
                    irAHome();

                } else if (etiqueta == "HOME") {
                    // IZQUIERDA+DERECHA a la vez. Si hay una articulación en
                    // pantalla, resetea SOLO esa en vez del home global.
                    if (pantalla >= 1 && pantalla <= 6) {
                        Serial.printf("RESET solicitado para articulacion %d\n", pantalla);
                        central.sendReset(pantalla);
                    } else {
                        Serial.println("HOME solicitado");
                        irAHome();
                    }

                } else if (etiqueta == "STEP") {
                    // Recorren la secuencia StepSequence en ciclo cerrado.
                    if (pantalla >= 1 && pantalla <= 6) {
                        if (valor == "UP") stepSequence.advance();
                        else if (valor == "DOWN") stepSequence.retreat();

                        stepSequence.rememberForJoint(pantalla);
                        escalonCambio = true;
                        nano.send("ESC:" + String((float)stepSequence.current()) + "\n");
                    }
                }
            }
        }

        // Actualizar pantalla Nextion con ángulos actuales (cada 300 ms).
        // A 9600 baudios, refrescar solo lo que cambió mantiene el tráfico
        // muy por debajo del límite del UART.
        if (millis() - lastDisplayUpdate > 300) {
            for (int i = 0; i < 6; i++) {
                float actual = central.lastAngleDeg(i + 1);
                if (fabs(actual - angulosEnviados[i]) > 0.005f) {
                    nextion.sendFloat("x" + String(i + 1), actual, 2);
                    nextion.updateDegreeBar("j" + String(i + 1), (int)actual);
                    angulosEnviados[i] = actual;
                }
            }
            if (escalonCambio) {
                escalonCambio = false;
                nextion.sendFloat("x20", stepSequence.current(), 2);
            }
            lastDisplayUpdate = millis();
        }

        // 4 ms: reduce la latencia de sondeo del enlace auxiliar (POS/BTN/SETA).
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

//------------------------------------------------------------------------
//  SETUP
//------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Ventana OTA de arranque: si en los primeros 30s alguien se conecta al
    // WiFi propio del mando, se queda en modo actualización y nunca vuelve
    // de aquí. Si nadie se conecta, sigue el arranque normal de abajo.
    ota.bootWindowCheck();

    // Deshabilitar el Task Watchdog de Core 1: WiFi.mode() + esp_now_init()
    // pueden tardar más de 5s en algunos arranques del ESP32-S3, antes de
    // que loop() pueda alimentar el watchdog. No es un cuelgue real.
    disableCore1WDT();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Desactivar modem-sleep: con el ahorro de energía activo, esp_now_send()
    // puede tener que despertar el radio primero, lo bastante lento como
    // para disparar el Interrupt Watchdog.
    WiFi.setSleep(false);
    Serial.print("MAC del Mando: ");
    Serial.println(WiFi.macAddress());  // anotar y añadir al Central

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error iniciando ESP-NOW");
        return;
    }

    central.begin();
    stepSequence.begin();

    nextion.begin(nextionSerial, mandoConfig::nextionBaudRate, mandoConfig::nextionRxPin,
                  mandoConfig::nextionTxPin, mandoConfig::nextionTxBufferSize);
    nano.begin(auxSerial, mandoConfig::auxBaudRate, mandoConfig::auxRxPin, mandoConfig::auxTxPin);
    delay(500);

    // Valores iniciales en pantalla
    nextion.sendFloat("x0", 0.0f, 2);
    nextion.sendFloat("x20", stepSequence.current(), 2);
    for (int i = 1; i <= 6; i++) {
        nextion.sendFloat("x1" + String(i), velocidades[i - 1], 0);
    }

    // Tareas FreeRTOS. esp_now_send() se llama directo desde nextionTask
    // (ver enviarSetpointCentral), sin cola ni salto de tarea/core intermedio.
    // Stack de nextionTask ampliado: hace muchas operaciones con String y
    // además llama a esp_now_send().
    xTaskCreatePinnedToCore(touchTask,   "TouchTask",   4096,  NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(nextionTask, "NextionTask", 16384, NULL, 2, NULL, 0);

    Serial.println("Mando ESP-NOW listo");
}

//------------------------------------------------------------------------
//  LOOP (Core 1) — vacío; toda la lógica está en tareas de Core 0.
//------------------------------------------------------------------------
void loop() {
    delay(100);  // alimenta el WDT del loopTask sin bloquear nada útil
}
