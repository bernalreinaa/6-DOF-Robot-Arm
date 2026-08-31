#include "CentralOta.h"
#include <WiFi.h>
#include <Update.h>

namespace {
constexpr unsigned long kOtaBootWindowMs = 30000UL;      // 30s para conectarse al arrancar
constexpr unsigned long kOtaUploadTimeoutMs = 600000UL;  // 10 min de margen para subir el firmware
const char* kOtaApName = "Central_OTA";
const char* kOtaPassword = "brazo1234";
}  // namespace

CentralOta::CentralOta(StatusBeacon& beacon) : beacon_(beacon) {}

void CentralOta::enterOtaMode() {
    Serial.println(F("Modo OTA: preparando punto de acceso..."));

    WiFi.mode(WIFI_AP);
    WiFi.softAP(kOtaApName, kOtaPassword);
    IPAddress ip = WiFi.softAPIP();

    Serial.println(F("Red WiFi   : Central_OTA"));
    Serial.println(F("Contraseña : brazo1234"));
    Serial.print(F("Abrir en el navegador: http://")); Serial.println(ip);

    server_.on("/", HTTP_GET, [this]() {
        server_.send(200, "text/html",
            "<html><body style='font-family:sans-serif'>"
            "<h2>OTA - Central (ESP32-S3)</h2>"
            "<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='update'> "
            "<input type='submit' value='Actualizar firmware'>"
            "</form></body></html>");
    });
    server_.on("/update", HTTP_POST, [this]() {
        bool ok = !Update.hasError();
        server_.sendHeader("Connection", "close");
        server_.send(200, "text/plain", ok ? "OK, reiniciando..." : "ERROR al actualizar");
        delay(500);
        ESP.restart();
    }, [this]() {
        HTTPUpload& upload = server_.upload();
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
    server_.begin();

    active_ = true;
    startMs_ = millis();
}

void CentralOta::bootWindowCheck() {
    Serial.println(F("Ventana OTA de arranque: 30s para conectarse..."));
    Serial.println(F("Red WiFi: Central_OTA"));

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
        delay(25);
        beacon_.setYellow(true);
        delay(25);
        beacon_.setYellow(false);
    }

    if (connected) {
        Serial.println(F("Cliente conectado: entrando en modo OTA"));
        enterOtaMode();  // deja active_=true; loop() atiende el resto via poll()
        return;
    }

    Serial.println(F("Sin conexion en 30s: arranque normal"));
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_MODE_STA);
}

void CentralOta::poll() {
    server_.handleClient();
    if (millis() - startMs_ > kOtaUploadTimeoutMs) {
        Serial.println(F("OTA sin usar, reiniciando a modo normal..."));
        ESP.restart();
    }
}
