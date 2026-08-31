// NextionUi — envoltorio del enlace serie con la pantalla táctil Nextion:
// helpers para fijar el valor de un componente, actualizar una barra de
// progreso en grados, y cambiar de página.
#pragma once
#include <Arduino.h>

class NextionUi {
public:
    void begin(HardwareSerial& serialPort, uint32_t baudRate, int rxPin, int txPin, size_t txBufferSize);

    // Fija objName.val al valor real multiplicado por 10^decimales (formato
    // entero que espera el componente Nextion).
    void sendFloat(const String& objName, float realValue, int decimals);

    // Actualiza una barra de progreso (0-100%) a partir de un ángulo 0-360°.
    void updateDegreeBar(const String& objName, int degrees);

    void changePage(const String& pageName);

    bool available();
    int read();
    void setTimeout(unsigned long timeoutMs);
    String readStringUntil(char terminator);

private:
    HardwareSerial* serial_ = nullptr;

    // Terminador de comando Nextion: tres bytes 0xFF.
    void sendTerminator();
};
