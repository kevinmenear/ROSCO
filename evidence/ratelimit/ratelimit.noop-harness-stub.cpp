// STUB -- NOT A TRANSLATION. The unit as a NO-OP: it reads no argument, writes
// no output and returns 0.0. This is the differential harness's red test -- the
// perturbation the green at harness/ratelimit.json must be able to fail on.
//
// THE `saturate_c(` BELOW IS LOAD-BEARING AND IS NOT DEAD CODE TO DELETE.
// `vit/test_validate.generate_callee_bridges` finds this unit's callees by
// running a regex over THIS FILE's text, and the generated Makefile compiles and
// links `<stem>_callees.o` only `if callee_bridge`. A stub that names no callee
// therefore produces no bridge, and `harness.sh` has already dropped
// `saturate.cpp.o` from LIBS in its favour -- so the link dies on
// `undefined reference to saturate_c` and a red test that fails to BUILD is not
// a red test (unit #45, and unit #42 one shape over). The guard is a value no
// case can present, so the call never executes.
#include "vit_types.h"

double ratelimit(double inputSignal, double minRate, double maxRate, double DT, int32_t reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue) {
    (void)inputSignal; (void)minRate; (void)maxRate; (void)DT; (void)reset;
    (void)rlP; (void)inst; (void)has_ResetValue; (void)ResetValue;
    if (DT == -1.7976931348623157e308) {
        return saturate_c(inputSignal, minRate, maxRate);
    }
    return 0.0;
}
