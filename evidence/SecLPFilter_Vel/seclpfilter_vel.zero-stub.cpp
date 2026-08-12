// ZERO STUB -- not a translation. Reads no argument, writes constants.
// The INPUT to a measurement (X4): if the kernel still passes with this in
// place, the comparison cannot fail and the green above means nothing.
#include "vit_types.h"

double SecLPFilter_Vel(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)InputSignal; (void)DT; (void)CornerFreq; (void)Damp;
    (void)iStatus; (void)reset; (void)has_InitialValue; (void)InitialValue;
    const int i = *inst - 1;
    FP->lpfV_OutputSignalLast1[i] = 0.0;
    FP->lpfV_OutputSignalLast2[i] = 0.0;
    FP->lpfV_InputSignalLast1[i] = 0.0;
    FP->lpfV_InputSignalLast2[i] = 0.0;
    FP->lpfV_a2[i] = 0.0;
    FP->lpfV_a1[i] = 0.0;
    FP->lpfV_a0[i] = 0.0;
    FP->lpfV_b2[i] = 0.0;
    FP->lpfV_b1[i] = 0.0;
    FP->lpfV_b0[i] = 0.0;
    *inst = *inst + 1;
    return 0.0;
}
