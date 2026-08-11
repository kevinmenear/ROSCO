// ZERO STUB -- reads no input except `inst`, writes no filter state, returns 0.0.
// The liveness test for the kernel: if this PASSES, the 62-case window cannot
// tell the real translation from a function that computes nothing.
#include "vit_types.h"

double LPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)InputSignal; (void)DT; (void)CornerFreq; (void)FP;
    (void)iStatus; (void)reset; (void)has_InitialValue; (void)InitialValue;
    *inst = *inst + 1;
    return 0.0;
}
