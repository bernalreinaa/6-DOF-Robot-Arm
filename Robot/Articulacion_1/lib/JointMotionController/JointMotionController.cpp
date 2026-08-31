#include "JointMotionController.h"
#include <math.h>

JointMotionController::JointMotionController(const JointMechanics& mech, JointTuning tuning, StepperDriver& stepper)
    : mech_(mech), tuning_(tuning), stepper_(stepper) {
    // Pasos de motor por grado de SALIDA (incluye la reductora completa).
    float stepsPerOutputRev = (float)mech_.motorStepsPerRev * (float)mech_.driverMicrosteps * mech_.ratio;
    stepsPerOutputDeg_ = stepsPerOutputRev / 360.0f;
}

double JointMotionController::updateFeedback(double rawAccumulatorDeg) {
    if (mech_.encoderOnOutputSide) {
        // El AS5600 va geared directamente a la salida (encoderGearRatio),
        // independiente de la reductora del motor (ratio): el acumulador ya
        // queda en grados de SALIDA.
        accumulatorDeg_ = rawAccumulatorDeg / mech_.encoderGearRatio;
        outputAngleDeg_ = angleMath::wrapTo360(accumulatorDeg_ - offsetDeg_);
    } else {
        // El AS5600 mide el eje del motor, ANTES de la reductora: el
        // acumulador queda en grados de MOTOR: hay que dividir por "ratio"
        // para obtener el ángulo de salida.
        accumulatorDeg_ = rawAccumulatorDeg;
        outputAngleDeg_ = angleMath::wrapTo360(accumulatorDeg_ / mech_.ratio - offsetDeg_);
    }
    return outputAngleDeg_;
}

void JointMotionController::requestReset() {
    if (mech_.encoderOnOutputSide) {
        offsetDeg_ = angleMath::wrapTo360(accumulatorDeg_);
    } else {
        offsetDeg_ = angleMath::wrapTo360(accumulatorDeg_ / mech_.ratio);
    }
}

void JointMotionController::beginMove(float setpointDeg) {
    double deltaOut = angleMath::safeAngularDelta(outputAngleDeg_, setpointDeg,
                                                    tuning_.limitInfDeg, tuning_.limitSupDeg);
    double deltaAccum = mech_.encoderOnOutputSide ? deltaOut : deltaOut * mech_.ratio;
    targetAccumulatorDeg_ = accumulatorDeg_ + deltaAccum;

    // Compensación de backlash: al cambiar de sentido respecto al último
    // movimiento, la salida no se mueve hasta recorrer backlashOutDeg de
    // holgura mecánica — se extiende el objetivo esa cantidad (equivalente
    // en el espacio del acumulador) para compensarlo. Inerte si
    // backlashOutDeg == 0 (mayoría de articulaciones).
    double backlashAccumDeg = mech_.encoderOnOutputSide ? mech_.backlashOutDeg
                                                          : mech_.backlashOutDeg * mech_.ratio;
    int8_t curDir = (deltaOut >= 0.0) ? 1 : -1;
    if (lastDir_ != 0 && curDir != lastDir_ && backlashAccumDeg > 0.0) {
        targetAccumulatorDeg_ += (double)curDir * backlashAccumDeg;
    }
    lastDir_ = curDir;

    integral_ = 0.0f;
    prevError_ = 0.0f;
    okCount_ = 0;
}

bool JointMotionController::stepOnce(float velocityScale) {
    double eAccum = targetAccumulatorDeg_ - accumulatorDeg_;
    double eOutput = mech_.encoderOnOutputSide ? eAccum : eAccum / mech_.ratio;
    float ef = (float)eAccum;

    if (fabs(eOutput) <= tuning_.tolDeg) {
        okCount_++;
        if (okCount_ >= 5) {
            return true;  // posición alcanzada
        }
    } else {
        okCount_ = 0;
    }

    float dtS = mech_.controlPeriodMs * 1e-3f;
    float derivative = (ef - prevError_) / dtS;
    prevError_ = ef;

    if (fabs(eOutput) > tuning_.slowZoneDeg) {
        integral_ += ef * dtS;
        integral_ = constrain(integral_, -mech_.integralMax, mech_.integralMax);
    } else {
        integral_ *= 0.5f;  // reducir integral cerca del objetivo
    }

    float velCmd = tuning_.kp * ef + tuning_.ki * integral_ + tuning_.kd * derivative;

    angleMath::VelocityProfileConfig profileCfg{
        tuning_.maxVelDegS, tuning_.cruiseVelDegS, tuning_.approachVelDegS, tuning_.minVelDegS,
        tuning_.tolDeg, tuning_.slowZoneDeg, tuning_.approachZoneDeg};
    float maxAllowedVel = angleMath::velocityProfile(fabs(eOutput), profileCfg) * velocityScale;
    velCmd = constrain(velCmd, -maxAllowedVel, maxAllowedVel);

    if (fabs(velCmd) > 1.0f) {
        float stepsPerSecond = fabs(velCmd) * stepsPerOutputDeg_;
        if (stepsPerSecond > 0.1f) {
            float pulseIntervalUs = 1000000.0f / stepsPerSecond;

            bool dirForward = (velCmd > 0);
            if (mech_.invertDir) dirForward = !dirForward;

            stepper_.setDirection(dirForward, mech_.dirSetupUs);
            stepper_.stepHigh(mech_.pulseHighUs);

            if (pulseIntervalUs > (mech_.pulseHighUs + mech_.pulseLowUs + 100)) {
                uint32_t remainingDelay = (uint32_t)(pulseIntervalUs - mech_.pulseHighUs - mech_.pulseLowUs);
                stepper_.stepLow(remainingDelay);
            } else {
                stepper_.stepLow(mech_.pulseLowUs);
            }

            static unsigned long lastDebugMs = 0;
            if (millis() - lastDebugMs > 500) {
                Serial.printf("Control: e_out=%.2f vel_cmd=%.1f sps=%.1f\r\n",
                              eOutput, velCmd, stepsPerSecond);
                lastDebugMs = millis();
            }
        }
    }

    return false;
}
