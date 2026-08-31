#include "JointStorage.h"
#include <Preferences.h>

void JointStorage::load(JointTuning& tuning) {
    Preferences prefs;
    prefs.begin("joint_p", true);  // true = solo lectura
    tuning.kp                 = prefs.getFloat("kp", tuning.kp);
    tuning.ki                 = prefs.getFloat("ki", tuning.ki);
    tuning.kd                 = prefs.getFloat("kd", tuning.kd);
    tuning.maxVelDegS         = prefs.getFloat("maxvel", tuning.maxVelDegS);
    tuning.cruiseVelDegS      = prefs.getFloat("cruisevel", tuning.cruiseVelDegS);
    tuning.approachVelDegS    = prefs.getFloat("apprvel", tuning.approachVelDegS);
    tuning.minVelDegS         = prefs.getFloat("minvel", tuning.minVelDegS);
    tuning.tolDeg             = prefs.getFloat("tol", tuning.tolDeg);
    tuning.slowZoneDeg        = prefs.getFloat("slowzone", tuning.slowZoneDeg);
    tuning.approachZoneDeg    = prefs.getFloat("apprzone", tuning.approachZoneDeg);
    tuning.limitInfDeg        = prefs.getDouble("liminf", tuning.limitInfDeg);
    tuning.limitSupDeg        = prefs.getDouble("limsup", tuning.limitSupDeg);
    prefs.end();
    Serial.println(F("Parametros cargados desde flash (si existian; si no, se usan los del firmware)"));
}

void JointStorage::save(const JointTuning& tuning) {
    Preferences prefs;
    prefs.begin("joint_p", false);  // false = lectura/escritura
    prefs.putFloat("kp", tuning.kp);
    prefs.putFloat("ki", tuning.ki);
    prefs.putFloat("kd", tuning.kd);
    prefs.putFloat("maxvel", tuning.maxVelDegS);
    prefs.putFloat("cruisevel", tuning.cruiseVelDegS);
    prefs.putFloat("apprvel", tuning.approachVelDegS);
    prefs.putFloat("minvel", tuning.minVelDegS);
    prefs.putFloat("tol", tuning.tolDeg);
    prefs.putFloat("slowzone", tuning.slowZoneDeg);
    prefs.putFloat("apprzone", tuning.approachZoneDeg);
    prefs.putDouble("liminf", tuning.limitInfDeg);
    prefs.putDouble("limsup", tuning.limitSupDeg);
    prefs.end();
    Serial.println(F("Parametros guardados en flash (persisten tras reiniciar/reflashear)"));
}
