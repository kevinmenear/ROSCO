// ZERO-OUTPUT STUB -- not a translation. Identical to the shipped translation
// in every state write (the four history slots, the six coefficients, the
// history shift, the instance increment) EXCEPT that the filter expression is
// replaced by the constant 0.0.
//
// It measures WINDOW LIVENESS rather than translation correctness: a captured
// case whose InputSignal and whose four history slots are all zero returns 0.0
// from the real filter too, so such a case cannot tell this stub from the
// translation. The count of cases it FAILS is the count of cases the kernel's
// arithmetic is actually observable in.
#include "vit_types.h"

double SecLPFilter_Vel(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    const int i = *inst - 1;

    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    if ((iStatus == 0) || (reset != 0)) {
        FP->lpfV_OutputSignalLast1[i] = InitialValue_;
        FP->lpfV_OutputSignalLast2[i] = InitialValue_;
        FP->lpfV_InputSignalLast1[i] = InitialValue_;
        FP->lpfV_InputSignalLast2[i] = InitialValue_;
        FP->lpfV_a2[i] = (DT * DT) * (CornerFreq * CornerFreq) + 4.0 + 4.0 * Damp * CornerFreq * DT;
        FP->lpfV_a1[i] = 2.0 * (DT * DT) * (CornerFreq * CornerFreq) - 8.0;
        FP->lpfV_a0[i] = (DT * DT) * (CornerFreq * CornerFreq) + 4.0 - 4.0 * Damp * CornerFreq * DT;
        FP->lpfV_b2[i] = 2.0 * DT * (CornerFreq * CornerFreq);
        FP->lpfV_b1[i] = 0.0;
        FP->lpfV_b0[i] = -(2.0 * DT * (CornerFreq * CornerFreq));
    }

    const double SecLPFilter_Vel_result = 0.0;   // <- the whole of the perturbation

    FP->lpfV_InputSignalLast2[i] = FP->lpfV_InputSignalLast1[i];
    FP->lpfV_InputSignalLast1[i] = InputSignal;
    FP->lpfV_OutputSignalLast2[i] = FP->lpfV_OutputSignalLast1[i];
    FP->lpfV_OutputSignalLast1[i] = SecLPFilter_Vel_result;

    *inst = *inst + 1;
    return SecLPFilter_Vel_result;
}
