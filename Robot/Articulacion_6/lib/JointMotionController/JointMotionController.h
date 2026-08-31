// JointMotionController — lógica de control de posición de UNA articulación:
// combina la lectura acumulada del encoder (en su espacio físico propio, ver
// JointConfig::encoderOnOutputSide) con un control PID + perfil de velocidad
// suave para generar los pulsos de paso, evitando siempre la zona prohibida.
//
// Es el equivalente modular de las tareas "tarea_encoder"/"tarea_nema" del
// firmware original, separando el CÁLCULO (aquí) de la ORQUESTACIÓN de
// tareas FreeRTOS (que queda en src/main.cpp).
#pragma once
#include <Arduino.h>
#include "AngleMath.h"
#include "StepperDriver.h"

// Parámetros ajustables en caliente (PID, perfil de velocidad, zona
// prohibida). Se inicializan desde JointConfig pero pueden cambiar en
// tiempo de ejecución (ver EspNowProtocol::TuningParams) y persisten en
// flash (ver JointStorage).
struct JointTuning {
    float kp, ki, kd;
    float maxVelDegS, cruiseVelDegS, approachVelDegS, minVelDegS;
    float tolDeg, slowZoneDeg, approachZoneDeg;
    double limitInfDeg, limitSupDeg;
};

// Parámetros fijos de la mecánica (no cambian en caliente).
struct JointMechanics {
    float ratio;
    int   motorStepsPerRev;
    int   driverMicrosteps;
    bool  encoderOnOutputSide;
    float encoderGearRatio;
    bool  invertDir;
    float backlashOutDeg;
    // Anchos de pulso/dir del TB6600 (µs).
    uint32_t pulseHighUs;
    uint32_t pulseLowUs;
    uint32_t dirSetupUs;
    float integralMax;      // anti-windup
    float controlPeriodMs;  // DT_MS del lazo PID
};

// Valores por defecto habituales de pulseHighUs/pulseLowUs/dirSetupUs/
// integralMax/controlPeriodMs, iguales en las 6 articulaciones — usar al
// construir JointMechanics en main.cpp:
//   JointMechanics{ratio, motorStepsPerRev, driverMicrosteps,
//                  encoderOnOutputSide, encoderGearRatio, invertDir,
//                  backlashOutDeg, kDefaultPulseHighUs, kDefaultPulseLowUs,
//                  kDefaultDirSetupUs, kDefaultIntegralMax, kDefaultControlPeriodMs}
constexpr uint32_t kDefaultPulseHighUs = 5;
constexpr uint32_t kDefaultPulseLowUs  = 5;
constexpr uint32_t kDefaultDirSetupUs  = 5;
constexpr float    kDefaultIntegralMax = 100.0f;
constexpr float    kDefaultControlPeriodMs = 1.0f;

class JointMotionController {
public:
    JointMotionController(const JointMechanics& mech, JointTuning tuning, StepperDriver& stepper);

    // --- Realimentación del encoder (llamar desde la tarea de encoder) ---
    // rawAccumulatorDeg: turns*360 + ángulo_módulo_360 crudo del sensor
    // (espacio físico del propio AS5600, sin dividir por ninguna reductora).
    // Devuelve el ángulo de SALIDA (0-360°) ya calculado.
    double updateFeedback(double rawAccumulatorDeg);

    // Recalibra el offset para que la posición física actual pase a ser 0°
    // (no mueve el motor).
    void requestReset();

    double outputAngleDeg() const { return outputAngleDeg_; }

    bool isInForbiddenZone(double angleDeg) const {
        return angleMath::isInForbiddenZone(angleDeg, tuning_.limitInfDeg, tuning_.limitSupDeg);
    }

    // --- Planificación y ejecución del movimiento (tarea de motor) ---
    // Arranca (o re-planifica sobre la marcha) un movimiento hacia
    // setpointDeg, calculando el camino seguro más corto y reiniciando el
    // estado del PID.
    void beginMove(float setpointDeg);

    // Una iteración del lazo de control (PID + generación de pulso si
    // corresponde). velocityScale es el % de velocidad (0.05-1.0) pedido
    // para este movimiento. Devuelve true cuando la posición se considera
    // alcanzada (igual que el "ok_cnt >= 5" del firmware original).
    bool stepOnce(float velocityScale);

    JointTuning& tuning() { return tuning_; }
    const JointTuning& tuning() const { return tuning_; }

private:
    JointMechanics mech_;
    JointTuning tuning_;
    StepperDriver& stepper_;

    // Precalculado a partir de mech_/ratio: pasos de motor por grado de
    // SALIDA (incluye la reductora), usado para convertir la velocidad de
    // control (grados/s) en pasos/s.
    float stepsPerOutputDeg_;

    // Estado de realimentación
    double offsetDeg_ = 0.0;
    double accumulatorDeg_ = 0.0;   // en el espacio elegido por encoderOnOutputSide (ver .cpp)
    double outputAngleDeg_ = 0.0;

    // Estado de movimiento
    double targetAccumulatorDeg_ = 0.0;
    int8_t lastDir_ = 0;   // para compensación de backlash: 1=CW, -1=CCW, 0=sin referencia

    // Estado del PID (se reinicia en cada beginMove/replanificación)
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    int   okCount_ = 0;
};
