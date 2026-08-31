#include "AngleMath.h"
#include <math.h>

namespace angleMath {

double wrapTo180(double angleDeg) {
    while (angleDeg > 180.0) angleDeg -= 360.0;
    while (angleDeg <= -180.0) angleDeg += 360.0;
    return angleDeg;
}

double wrapTo360(double angleDeg) {
    double a = fmod(angleDeg, 360.0);
    if (a < 0) a += 360.0;
    return a;
}

bool isInForbiddenZone(double angleDeg, double limitInfDeg, double limitSupDeg) {
    if (limitInfDeg == limitSupDeg) return false;  // sin zona prohibida definida
    angleDeg = wrapTo360(angleDeg);
    if (limitInfDeg < limitSupDeg) {
        return (angleDeg >= limitInfDeg && angleDeg <= limitSupDeg);
    } else {
        // La zona prohibida envuelve el 0°
        return (angleDeg >= limitInfDeg || angleDeg <= limitSupDeg);
    }
}

// ============================================================
// safeAngularDelta(current, target)
//
// Calcula el desplazamiento angular óptimo (en grados) para llevar el motor
// desde 'current' hasta 'target', evitando pasar por la zona mecánicamente
// prohibida [limitInfDeg, limitSupDeg].
//
// Para ir de A a B en un círculo hay exactamente dos caminos posibles (CW y
// CCW); se simula cada uno en pasos de 5° comprobando si algún punto
// intermedio cae en la zona prohibida, y se elige el camino libre (o el más
// corto si ambos lo están, o si ambos están bloqueados por una zona mal
// configurada).
// ============================================================
double safeAngularDelta(double currentDeg, double targetDeg,
                         double limitInfDeg, double limitSupDeg) {
    double current = wrapTo360(currentDeg);
    double target  = wrapTo360(targetDeg);

    double deltaCw  = fmod((target - current + 360.0), 360.0);   // siempre >= 0
    double deltaCcw = fmod((current - target + 360.0), 360.0);   // siempre >= 0

    auto crossesForbidden = [&](double from, double to, bool cw) -> bool {
        double a = from;
        for (int i = 0; i < 360; i += 5) {
            if (isInForbiddenZone(a, limitInfDeg, limitSupDeg)) return true;
            a = cw ? fmod(a + 5.0, 360.0) : fmod(a - 5.0 + 360.0, 360.0);
            if (fabs(wrapTo180(a - to)) < 2.5) break;  // llegada con margen
        }
        return false;
    };

    bool cwForbidden  = crossesForbidden(current, target, true);
    bool ccwForbidden = crossesForbidden(current, target, false);

    if (cwForbidden && !ccwForbidden) {
        return -deltaCcw;
    } else if (!cwForbidden && ccwForbidden) {
        return deltaCw;
    } else if (!cwForbidden && !ccwForbidden) {
        return (deltaCw <= deltaCcw) ? deltaCw : -deltaCcw;
    } else {
        // Configuración inesperada (zona muy amplia o límites erróneos):
        // se toma igualmente el camino más corto para minimizar el riesgo.
        return (deltaCw <= deltaCcw) ? deltaCw : -deltaCcw;
    }
}

float velocityProfile(float errorDegAbs, const VelocityProfileConfig& cfg) {
    if (errorDegAbs <= cfg.tolDeg) {
        return 0.0f;
    } else if (errorDegAbs <= cfg.slowZoneDeg) {
        float t = errorDegAbs / cfg.slowZoneDeg;
        return cfg.minVelDegS + (cfg.approachVelDegS - cfg.minVelDegS) * (t * t);
    } else if (errorDegAbs <= cfg.approachZoneDeg) {
        float t = (errorDegAbs - cfg.slowZoneDeg) / (cfg.approachZoneDeg - cfg.slowZoneDeg);
        return cfg.approachVelDegS + (cfg.cruiseVelDegS - cfg.approachVelDegS) * t;
    } else {
        return cfg.cruiseVelDegS;
    }
}

}  // namespace angleMath
