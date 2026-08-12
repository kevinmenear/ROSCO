// RED TEST for the differential harness: the unit as a no-op. Reads no
// argument, writes no FP field, does not advance `inst`, returns 0.0.
// It must fail every case, and the mismatch list must NAME every output the
// unit is supposed to write -- that list is what says the harness was looking
// at the right things (unit #5's rule).
#include "vit_types.h"
double NotchFilter(double InputSignal, double DT, double omega, double betaNum, double betaDen, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)InputSignal; (void)DT; (void)omega; (void)betaNum; (void)betaDen;
    (void)FP; (void)iStatus; (void)reset; (void)inst;
    (void)has_InitialValue; (void)InitialValue;
    return 0.0;
}
