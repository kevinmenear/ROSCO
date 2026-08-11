// STUB, not a translation. The unit's ONLY output is its LOGICAL result, so no
// single constant can fail every case -- the corpus now contains both answers.
// The complement does: it is wrong on every case, whichever way the reference
// answered, which is what makes `36 of 36 failed` a statement about the
// comparison rather than about the corpus being one-sided.
#include <cstdint>
int32_t NonDecreasing(double* Array, int n_Array) {
    int32_t r = 1;
    for (int I_DIFF = 1; I_DIFF <= n_Array - 1; ++I_DIFF) {
        if (Array[(I_DIFF + 1) - 1] - Array[I_DIFF - 1] <= 0.0) { r = 0; break; }
    }
    return r ? 0 : 1;
}
