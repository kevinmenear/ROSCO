// STUB, not a translation. Reads NO argument and returns .FALSE. unconditionally.
// This is the LIVENESS test: .FALSE. makes `.NOT. NonDecreasing(...)` true, the
// caller's IF body runs, and ErrVar%aviFAIL/ErrMsg change. If the kernel does NOT
// fail on this, the comparison is dead.
#include <cstdint>
int32_t NonDecreasing(double* Array, int n_Array) {
    (void)Array; (void)n_Array;
    return 0;
}
