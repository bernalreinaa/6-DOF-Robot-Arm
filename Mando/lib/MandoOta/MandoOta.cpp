#include "MandoOta.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

namespace {
constexpr unsigned long kOtaBootWindowMs = 30000UL;
constexpr unsigned long kOtaUploadTimeoutMs = 600000UL;
const char* kOtaApName = "Mando_OTA";
const char* kOtaPassword = "brazo1234";
}  // namespace

void MandoOta::enterOtaMode() {
    Serial.println(F("Modo OTA: preparando punto de acceso..."));

    WiFi.mode(WIFI_AP);
    WiFi.softAP(kOtaApName, kOtaPassword);
    IPAddress ip = WiFi.softAPIP();

    Serial.println(F("Red WiFi   : Mando_OTA"));
    Serial.println(F("Contraseña : brazo1234"));
    Serial.print(F("Abrir en el navegador: http://")); Serial.println(ip);

    WebServer server(80);
    server.on("/", HTTP_GET, [&server]() {
        server.send(200, "text/html",
            "<html><body style='font-family:sans-serif'>"
            "<h2>OTA - Mando</h2>"
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
    ESP.restart();
}

void MandoOta::bootWindowCheck() {
    Serial.println(F("Ventana OTA de arranque: 30s para conectarse..."));
    Serial.println(F("Red WiFi: Mando_OTA"));

    WiFi.mode(WIFI_AP);
    WiFi.softAP(kOtaApName, kOtaPassword);

    unsigned long startMs = millis();
    unsigned long lastNotice = 0;
    bool connected = false;
    while (millis() - startMs < kOtaBootWindowMs) {
        if (WiFi.softAPgetStationNum() > 0) {
            connected = true;
            break;
        }
        if (millis() - lastNotice >= 1000) {
            lastNotice = millis();
            int secondsLeft = (int)((kOtaBootWindowMs - (millis() - startMs)) / 1000) + 1;
            Serial.printf("OTA %ds...\n", secondsLeft);
        }
        delay(50);
    }

    if (connected) {
        Serial.println(F("Cliente conectado: entrando en modo OTA"));
        enterOtaMode();
    }

    Serial.println(F("Sin conexion en 30s: arranque normal"));
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
}
