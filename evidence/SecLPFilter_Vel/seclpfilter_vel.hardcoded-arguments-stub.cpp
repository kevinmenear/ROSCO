// HARDCODED-ARGUMENT STUB -- not a translation. Identical to the shipped
// translation except that it IGNORES four of the ten arguments and writes the
// literals the simulation happens to carry at the one call site:
//     CornerFreq -> 2*PI/CC_ActTau = 2*PI/20
//     Damp       -> REAL(1.0,DbKi)
//     has_InitialValue / InitialValue -> never passed by the call site
// It is the INPUT to a measurement of what the KERNEL cannot constrain.
#include "vit_types.h"

double SecLPFilter_Vel(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)CornerFreq; (void)Damp; (void)has_InitialValue; (void)InitialValue;
    const double CornerFreq_ = 2.0 * 3.1415926535897932384626433832795 / 20.0;
    const double Damp_ = 1.0;
    const int i = *inst - 1;

    double InitialValue_ = InputSignal;

    if ((iStatus == 0) || (reset != 0)) {
        FP->lpfV_OutputSignalLast1[i] = InitialValue_;
        FP->lpfV_OutputSignalLast2[i] = InitialValue_;
        FP->lpfV_InputSignalLast1[i] = InitialValue_;
        FP->lpfV_InputSignalLast2[i] = InitialValue_;
        FP->lpfV_a2[i] = (DT * DT) * (CornerFreq_ * CornerFreq_) + 4.0 + 4.0 * Damp_ * CornerFreq_ * DT;
        FP->lpfV_a1[i] = 2.0 * (DT * DT) * (CornerFreq_ * CornerFreq_) - 8.0;
        FP->lpfV_a0[i] = (DT * DT) * (CornerFreq_ * CornerFreq_) + 4.0 - 4.0 * Damp_ * CornerFreq_ * DT;
        FP->lpfV_b2[i] = 2.0 * DT * (CornerFreq_ * CornerFreq_);
        FP->lpfV_b1[i] = 0.0;
        FP->lpfV_b0[i] = -(2.0 * DT * (CornerFreq_ * CornerFreq_));
    }

    const double SecLPFilter_Vel_result =
        1.0 / FP->lpfV_a2[i] * (FP->lpfV_b2[i] * InputSignal
                                + FP->lpfV_b1[i] * FP->lpfV_InputSignalLast1[i]
                                + FP->lpfV_b0[i] * FP->lpfV_InputSignalLast2[i]
                                - FP->lpfV_a1[i] * FP->lpfV_OutputSignalLast1[i]
                                - FP->lpfV_a0[i] * FP->lpfV_OutputSignalLast2[i]);

    FP->lpfV_InputSignalLast2[i] = FP->lpfV_InputSignalLast1[i];
    FP->lpfV_InputSignalLast1[i] = InputSignal;
    FP->lpfV_OutputSignalLast2[i] = FP->lpfV_OutputSignalLast1[i];
    FP->lpfV_OutputSignalLast1[i] = SecLPFilter_Vel_result;

    *inst = *inst + 1;
    return SecLPFilter_Vel_result;
}
