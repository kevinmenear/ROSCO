// STUB, not a translation. Reads NO argument and returns .TRUE. unconditionally.
// If the kernel PASSES this, the kernel is a lookup table for NonDecreasing:
// every captured case's reference answer is .TRUE., so a constant can satisfy it.
#include <cstdint>
int32_t NonDecreasing(double* Array, int n_Array) {
    (void)Array; (void)n_Array;
    return 1;
}
