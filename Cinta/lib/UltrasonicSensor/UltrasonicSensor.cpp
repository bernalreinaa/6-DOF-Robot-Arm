#include "UltrasonicSensor.h"

namespace {
// Única instancia esperada en el proyecto (un solo HC-SR04): estado de la
// ISR como variables de módulo, más sencillo y ligero que un mecanismo de
// callback genérico para un caso de uso tan acotado.
volatile unsigned long g_echoRiseUs = 0;
volatile unsigned long g_echoFallUs = 0;
volatile bool g_echoPulseReady = false;      // true = ya hay un pulso completo (subida+bajada) sin leer
volatile bool g_echoWaitingFall = false;     // true entre la subida y la bajada de este pulso
int g_echoPin = -1;

void IRAM_ATTR isrEcho() {
    if (digitalRead(g_echoPin) == HIGH) {
        g_echoRiseUs = micros();
        g_echoWaitingFall = true;
        g_echoPulseReady = false;
    } else if (g_echoWaitingFall) {
        g_echoFallUs = micros();
        g_echoWaitingFall = false;
        g_echoPulseReady = true;
    }
}
}  // namespace

void UltrasonicSensor::begin(int trigPin, int echoPin, unsigned long timeoutUs, int medianSamples) {
    trigPin_ = trigPin;
    echoPin_ = echoPin;
    timeoutUs_ = timeoutUs;
    medianSamples_ = constrain(medianSamples, 1, kMaxMedianSamples);
    for (int i = 0; i < medianSamples_; i++) samples_[i] = -1.0f;
    sampleIndex_ = 0;

    g_echoPin = echoPin_;

    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(trigPin_, LOW);
    attachInterrupt(digitalPinToInterrupt(echoPin_), isrEcho, CHANGE);
}

float UltrasonicSensor::measureCm() {
    g_echoPulseReady = false;
    g_echoWaitingFall = false;  // por si el ciclo anterior se quedó a medias (sin eco)

    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    unsigned long waitStartUs = micros();
    unsigned long durationUs = 0;
    while ((unsigned long)(micros() - waitStartUs) < timeoutUs_) {
        if (g_echoPulseReady) {
            durationUs = (unsigned long)(g_echoFallUs - g_echoRiseUs);  // resta sin signo, segura ante overflow de micros()
            break;
        }
        vTaskDelay(1);
    }

    float rawCm = (durationUs == 0) ? -1.0f : (durationUs / 58.0f);  // fórmula estándar del HC-SR04
    samples_[sampleIndex_] = rawCm;
    sampleIndex_ = (sampleIndex_ + 1) % medianSamples_;

    float valid[kMaxMedianSamples];
    int validCount = 0;
    for (int i = 0; i < medianSamples_; i++) {
        if (samples_[i] >= 0.0f) valid[validCount++] = samples_[i];
    }
    if (validCount == 0) {
        return -1.0f;  // ninguna lectura válida reciente
    }

    // Insertion sort: validCount <= medianSamples_, muy pequeño.
    for (int i = 1; i < validCount; i++) {
        float key = valid[i];
        int j = i - 1;
        while (j >= 0 && valid[j] > key) {
            valid[j + 1] = valid[j];
            j--;
        }
        valid[j + 1] = key;
    }

    return valid[validCount / 2];
}
