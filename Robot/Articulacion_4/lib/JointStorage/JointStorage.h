// JointStorage — persistencia en flash (NVS, vía Preferences) de los
// parámetros ajustables de una articulación (PID, perfil de velocidad, zona
// prohibida), para no tener que reflashear el firmware cada vez que se
// ajusta un parámetro desde la app. Espacio de nombres propio ("joint_p"),
// independiente del que usa internamente el driver WiFi/ESP-NOW.
#pragma once
#include "JointMotionController.h"

class JointStorage {
public:
    // Carga en "tuning" los valores guardados (si existen); si no hay nada
    // guardado, deja "tuning" sin modificar (se conservan los valores de
    // fábrica de JointConfig).
    void load(JointTuning& tuning);

    // Guarda "tuning" en flash. LLAMAR SIEMPRE desde loop()/una tarea normal,
    // NUNCA desde el callback de recepción ESP-NOW: la escritura a flash
    // puede tardar varios ms y no debe bloquear la pila WiFi/ESP-NOW.
    void save(const JointTuning& tuning);
};
