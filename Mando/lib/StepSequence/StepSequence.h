// StepSequence — secuencia cíclica de incrementos de paso ("escalón") que
// recorren los botones físicos SUBIR/BAJAR del mando (0.01, 0.1, 0.5, 1.0,
// 1.5 ... 10.0). Ciclo cerrado: al pasar del último valor se vuelve al
// primero, y al bajar del primero se salta al último. El paso se recuerda
// POR ARTICULACIÓN (1-6): al cambiar de página se carga el que se estaba
// usando en esa articulación, en vez de arrastrar el de cualquier otra.
#pragma once
#include <Arduino.h>

class StepSequence {
public:
    void begin();

    float current() const { return currentStep_; }

    void advance();   // botón SUBIR: siguiente valor de la secuencia
    void retreat();   // botón BAJAR: valor anterior de la secuencia

    // Carga el paso recordado para esta articulación (1-6) como el actual.
    void recallForJoint(int jointId);
    // Guarda el paso actual como el recordado para esta articulación.
    void rememberForJoint(int jointId);

private:
    // La secuencia de valores vive en StepSequence.cpp (array de ámbito de
    // fichero, no miembro estático) para evitar cualquier problema de
    // enlazado con arrays "static constexpr" odr-usados en C++11/14.
    static constexpr int kStepsCount = 22;
    static constexpr int kDefaultIndex = 2;  // índice de 0.5, el paso por defecto

    float currentStep_ = 0.5f;  // begin() lo ajusta al valor real de kDefaultIndex
    int   currentIndex_ = kDefaultIndex;

    float stepPerJoint_[6]  = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    int   indexPerJoint_[6] = {kDefaultIndex, kDefaultIndex, kDefaultIndex,
                                kDefaultIndex, kDefaultIndex, kDefaultIndex};
};
