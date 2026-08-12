// STUB, not a translation. Reads NO argument and returns the constant .TRUE. --
// the WRONG answer at the instrumented call site. If the kernel FAILS this, the
// comparison is alive: the branch body writes CntrPar%PerfFileName.
#include <cstdint>
int32_t PathIsRelative(char* GivenFil, int len_GivenFil) {
    (void)GivenFil; (void)len_GivenFil;
    return 1;
}
