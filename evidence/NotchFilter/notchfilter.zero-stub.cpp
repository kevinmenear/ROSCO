// RED TEST for the NotchFilter kernel: reads no argument, writes no state,
// returns 0.0. If `vit verify` still passes, the kernel is comparing nothing.
#include "vit_types.h"
double NotchFilter(double InputSignal, double DT, double omega, double betaNum, double betaDen, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)InputSignal; (void)DT; (void)omega; (void)betaNum; (void)betaDen;
    (void)FP; (void)iStatus; (void)reset; (void)inst;
    (void)has_InitialValue; (void)InitialValue;
    return 0.0;
}
