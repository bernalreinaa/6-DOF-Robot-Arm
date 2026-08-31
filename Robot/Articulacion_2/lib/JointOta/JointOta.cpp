#include "JointOta.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

namespace {
constexpr unsigned long kOtaBootWindowMs = 30000UL;   // 30s para conectarse al arrancar
constexpr unsigned long kOtaUploadTimeoutMs = 600000UL;  // 10 min de margen para subir el firmware
const char* kOtaPassword = "brazo1234";
}  // namespace

JointOta::JointOta(JointDisplay& display, int jointId)
    : display_(display), jointId_(jointId) {
    apName_ = "Art" + String(jointId_) + "_OTA";
}

void JointOta::enterOtaMode() {
    Serial.println(F("Modo OTA: preparando punto de acceso..."));

    display_.showThreeLines("MODO OTA", "Conectando WiFi...", "");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName_.c_str(), kOtaPassword);
    IPAddress ip = WiFi.softAPIP();

    Serial.print(F("Red WiFi   : ")); Serial.println(apName_);
    Serial.println(F("Contraseña : brazo1234"));
    Serial.print(F("Abrir en el navegador: http://")); Serial.println(ip);

    display_.showOtaUploadInfo(apName_, ip);

    WebServer server(80);
    server.on("/", HTTP_GET, [&server]() {
        server.send(200, "text/html",
            "<html><body style='font-family:sans-serif'>"
            "<h2>OTA - Articulación</h2>"
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='update'> "
            "<input type='submit' value='Actualizar firmware'>"
            "</form></body></html>");
    });
    server.on("/update", HTTP_POST, [&server]() {
        bool ok = !Update.hasError();
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", ok ? "OK, reiniciando..." : "ERROR al actualizar");
        delay(500);
        ESP.restart();
    }, [&server]() {
        HTTPUpload &upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Recibiendo firmware: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) Serial.printf("Firmware OK: %u bytes\n", upload.totalSize);
            else Update.printError(Serial);
        }
    });
    server.begin();

    unsigned long startMs = millis();
    while (millis() - startMs < kOtaUploadTimeoutMs) {
        server.handleClient();
        delay(2);
    }

    Serial.println(F("OTA sin usar, reiniciando a modo normal..."));
    ESP.restart();  // más simple y fiable que reanudar STA/ESP-NOW a mano
}

void JointOta::bootWindowCheck() {
    Serial.println(F("Ventana OTA de arranque: 30s para conectarse..."));
    Serial.print(F("Red WiFi: ")); Serial.println(apName_);

    display_.showThreeLines(apName_.c_str(), "OTA 30s...", "Conectate para actualizar");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName_.c_str(), kOtaPassword);

    unsigned long startMs = millis();
    bool connected = false;
    while (millis() - startMs < kOtaBootWindowMs) {
        if (WiFi.softAPgetStationNum() > 0) {
            connected = true;
            break;
        }
        int secondsLeft = (int)((kOtaBootWindowMs - (millis() - startMs)) / 1000) + 1;
        display_.showOtaCountdown(apName_, secondsLeft);
        delay(200);
    }

    if (connected) {
        Serial.println(F("Cliente conectado: entrando en modo OTA"));
        enterOtaMode();  // no vuelve nunca de aqui (solo via ESP.restart())
    }

    Serial.println(F("Sin conexion en 30s: arranque normal"));
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
}
