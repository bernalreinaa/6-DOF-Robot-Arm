// AngleMath — funciones matemáticas puras para ángulos circulares (0-360°),
// zonas prohibidas y perfiles de velocidad suave. Sin estado, sin hardware:
// se puede probar de forma aislada y se reutiliza igual en las 6 articulaciones.
#pragma once

namespace angleMath {

// Normaliza un ángulo al rango (-180°, 180°].
double wrapTo180(double angleDeg);

// Normaliza un ángulo al rango [0°, 360°).
double wrapTo360(double angleDeg);

// ¿El ángulo (cualquier rango; se normaliza internamente) cae dentro de la
// zona mecánicamente prohibida [limitInfDeg, limitSupDeg]?
// - Si limitInfDeg == limitSupDeg: no hay zona prohibida definida (false).
// - Si limitInfDeg < limitSupDeg: zona prohibida normal (no cruza 0°).
// - Si limitInfDeg > limitSupDeg: zona prohibida que envuelve el 0°.
bool isInForbiddenZone(double angleDeg, double limitInfDeg, double limitSupDeg);

// Calcula el desplazamiento angular óptimo (grados, con signo) para llevar
// el motor desde 'currentDeg' hasta 'targetDeg' evitando cruzar la zona
// prohibida [limitInfDeg, limitSupDeg]. Positivo = sentido horario (CW),
// negativo = antihorario (CCW). Si ambos caminos cruzan la zona prohibida
// (configuración inesperada), se elige el camino más corto igualmente.
double safeAngularDelta(double currentDeg, double targetDeg,
                         double limitInfDeg, double limitSupDeg);

// Perfil de velocidad suave: dado el error absoluto (grados) y las zonas de
// control configuradas, devuelve la velocidad máxima permitida (°/s).
struct VelocityProfileConfig {
    float maxVelDegS;
    float cruiseVelDegS;
    float approachVelDegS;
    float minVelDegS;
    float tolDeg;
    float slowZoneDeg;
    float approachZoneDeg;
};

float velocityProfile(float errorDegAbs, const VelocityProfileConfig& cfg);

}  // namespace angleMath
