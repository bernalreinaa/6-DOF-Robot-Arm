#include "ConveyorDrive.h"
#include <math.h>

namespace {

// El callback de esp_timer es un puntero a función C plano; recibe "arg"
// (aquí, el pin PUL y el ancho de pulso empaquetados por begin() vía
// esp_timer_create_args_t.arg = this, y se leen los miembros a través del
// puntero recibido).
struct PulseTimerArgs {
    int pinPul;
    uint32_t pulseHighUs;
};

void pulseTimerCallback(void* arg) {
    auto* args = static_cast<PulseTimerArgs*>(arg);
    // Pulso corto en PUL. DIR ya se fijó ANTES de arrancar el temporizador
    // (en setVelocityPercent()), nunca aquí, para no arriesgarse a cambiar
    // de sentido a mitad de un pulso.
    digitalWrite(args->pinPul, HIGH);
    delayMicroseconds(args->pulseHighUs);
    digitalWrite(args->pinPul, LOW);
}

PulseTimerArgs g_pulseArgs;  // única cinta en el proyecto: instancia única basta

}  // namespace

void ConveyorDrive::begin(int pinPul, int pinDir, uint32_t pulseHighUs, uint32_t dirSetupUs,
                           float maxSpeedStepsPerSecond, bool invertDir) {
    pinPul_ = pinPul;
    pinDir_ = pinDir;
    pulseHighUs_ = pulseHighUs;
    dirSetupUs_ = dirSetupUs;
    invertDir_ = invertDir;
    maxSpeedStepsPerSecond_ = maxSpeedStepsPerSecond;

    pinMode(pinPul_, OUTPUT);
    pinMode(pinDir_, OUTPUT);

    g_pulseArgs.pinPul = pinPul_;
    g_pulseArgs.pulseHighUs = pulseHighUs_;

    const esp_timer_create_args_t timerArgs = {
        .callback = &pulseTimerCallback,
        .arg = &g_pulseArgs,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "conveyorPulseTimer",
        .skip_unhandled_events = true,
    };
    esp_err_t err = esp_timer_create(&timerArgs, &pulseTimer_);
    if (err != ESP_OK) {
        Serial.printf("Error creando el temporizador de pulsos (%d)\r\n", (int)err);
    }
}

void ConveyorDrive::setVelocityPercent(float velocityPercent) {
    velocityPercent = constrain(velocityPercent, -100.0f, 100.0f);

    // Un temporizador periódico de esp_timer no admite cambiar su periodo
    // en caliente: hay que pararlo (si estaba corriendo) antes de tocar DIR
    // o de volver a arrancarlo con el nuevo periodo.
    esp_timer_stop(pulseTimer_);  // no-op seguro si ya estaba parado

    float stepsPerSecond = fabs(velocityPercent) / 100.0f * maxSpeedStepsPerSecond_;
    if (fabs(velocityPercent) < 0.5f || stepsPerSecond < 1.0f) {
        currentStepsPerSecond_ = 0.0f;
        return;  // se queda parado (temporizador detenido, sin pulsos)
    }

    bool dirForward = (velocityPercent > 0);
    if (invertDir_) dirForward = !dirForward;
    digitalWrite(pinDir_, dirForward ? LOW : HIGH);
    delayMicroseconds(dirSetupUs_);

    uint64_t periodUs = (uint64_t)(1000000.0f / stepsPerSecond);
    if (periodUs < (pulseHighUs_ + 2)) {
        periodUs = pulseHighUs_ + 2;  // límite de seguridad (periodo mínimo utilizable)
    }

    esp_err_t err = esp_timer_start_periodic(pulseTimer_, periodUs);
    if (err != ESP_OK) {
        Serial.printf("Error al arrancar el temporizador de pulsos (%d)\r\n", (int)err);
        currentStepsPerSecond_ = 0.0f;
        return;
    }
    currentStepsPerSecond_ = stepsPerSecond;
}
