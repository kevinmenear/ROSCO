// ZERO STUB -- reads no argument, writes no state, returns 0.0. The kernel must
// FAIL this. It is the liveness test for the instrument: `62/62 IDENTICAL` and
// `62/62 IDENTICAL on inputs that cannot tell two translations apart` are the
// same sentence, and this is what separates them.
//
// The unit's arguments do NOT alias (unit #7's GetRoot shape), so a no-op is a
// liveness test here rather than a vacuity test.
#include "vit_types.h"

double NotchFilterSlopes(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_Moving, int32_t Moving, int has_InitialValue, double InitialValue) {
    (void)InputSignal; (void)DT; (void)CornerFreq; (void)Damp; (void)FP;
    (void)iStatus; (void)reset; (void)inst; (void)has_Moving; (void)Moving;
    (void)has_InitialValue; (void)InitialValue;
    return 0.0;
}
