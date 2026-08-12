// ZERO STUB -- X4 red test for the kernel, unit #18.
// Reads no argument and writes constants. If the kernel passes this, the
// comparison is not alive and 62/62 IDENTICAL means nothing.
#include "vit_types.h"

double SecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)InputSignal; (void)DT; (void)CornerFreq; (void)Damp; (void)FP;
    (void)iStatus; (void)reset; (void)has_InitialValue; (void)InitialValue;
    (void)inst;
    return 0.0;
}
