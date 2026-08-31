#include "StepSequence.h"

namespace {
// Por debajo de 0.5 solo hay dos valores discretos (0.01 y 0.1); de 0.5 a
// 10.0 sigue en incrementos de 0.5.
const float kSteps[] = {
    0.01f, 0.1f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 4.5f,
    5.0f,  5.5f, 6.0f, 6.5f, 7.0f, 7.5f, 8.0f, 8.5f, 9.0f, 9.5f, 10.0f,
};
}  // namespace

void StepSequence::begin() {
    currentIndex_ = kDefaultIndex;
    currentStep_ = kSteps[currentIndex_];
}

void StepSequence::advance() {
    currentIndex_ = (currentIndex_ + 1) % kStepsCount;
    currentStep_ = kSteps[currentIndex_];
}

void StepSequence::retreat() {
    currentIndex_ = (currentIndex_ - 1 + kStepsCount) % kStepsCount;
    currentStep_ = kSteps[currentIndex_];
}

void StepSequence::recallForJoint(int jointId) {
    if (jointId < 1 || jointId > 6) return;
    currentStep_ = stepPerJoint_[jointId - 1];
    currentIndex_ = indexPerJoint_[jointId - 1];
}

void StepSequence::rememberForJoint(int jointId) {
    if (jointId < 1 || jointId > 6) return;
    stepPerJoint_[jointId - 1] = currentStep_;
    indexPerJoint_[jointId - 1] = currentIndex_;
}
