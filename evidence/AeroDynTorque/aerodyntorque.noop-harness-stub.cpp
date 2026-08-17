// RED TEST for unit #48 `AeroDynTorque`: THE UNIT AS A NO-OP.
//
// Returns 0 and writes nothing. This is the stub the green result is certified
// against -- if the differential harness cannot tell this from the translation,
// its green says nothing about the translation either.
//
// THE `interp2d_c` CALL IS DELIBERATE AND IS NOT DEAD CODE. Unit #45 measured
// that `vit/test_validate.generate_callee_bridges` decides which callee bridges
// to emit by running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the
// generated Makefile compiles and links `<stem>_callees.o` only when at least
// one bridge was emitted. A stub that calls nothing therefore changes the
// harness's own LINK, and on the clean tree `scripts/harness.sh` has just
// dropped `interp2d.cpp.o` from LIBS in favour of the bridge -- so the link
// would die on `undefined reference to interp2d_c` and a red test that fails to
// BUILD is not a red test (unit #42).
//
// The guard is unsatisfiable on this corpus and on any corpus: `ErrVar` is
// always a real object here, so `ErrVar == nullptr` is never true, and the call
// never executes. Its only job is to be TEXT the bridge generator can see.
//
// DO NOT DELETE IT AS DEAD CODE.

#include "vit_types.h"

double AeroDynTorque(double RotSpeed, double BldPitch,
                     localvariables_view_t* LocalVar,
                     controlparameters_view_t* CntrPar,
                     performancedata_view_t* PerfData,
                     errorvariables_view_t* ErrVar) {
    if (ErrVar == nullptr) {
        // Unreachable. See the header: this exists so that the callee bridge is
        // emitted and the harness links.
        return interp2d_c(PerfData->Beta_vec, PerfData->n_Beta_vec,
                          PerfData->TSR_vec, PerfData->n_TSR_vec,
                          PerfData->Cp_mat, PerfData->n_Cp_mat_rows,
                          PerfData->n_Cp_mat_cols, BldPitch, RotSpeed, ErrVar);
    }
    (void)LocalVar;
    (void)CntrPar;
    return 0.0;
}
