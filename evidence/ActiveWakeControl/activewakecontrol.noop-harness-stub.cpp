// STUB -- NOT A TRANSLATION. The unit as a NO-OP: it reads no argument and
// writes no output. This is the differential harness's red test -- the
// perturbation the green at harness/ActiveWakeControl.json must be able to fail
// on.
//
// IT IS NOT EXPECTED TO FAIL EVERY CASE, and the shortfall is the measurement.
// The reference's whole body is one IF/ELSEIF chain over `CntrPar%AWC_Mode`
// with no ELSE, so every case whose mode is outside 1..5 has the reference
// writing nothing either -- a no-op is the CORRECT answer there. The number of
// cases this stub fails is therefore the number of cases that reach an arm at
// all, and it is recorded rather than predicted.
//
// THE FIVE `<callee>_c(` CALLS BELOW ARE LOAD-BEARING AND ARE NOT DEAD CODE TO
// DELETE. `vit/test_validate.generate_callee_bridges` finds this unit's callees
// by running a regex over THIS FILE's text, and the generated Makefile compiles
// and links `<stem>_callees.o` only `if callee_bridge`. On the CLEAN tree
// `harness.sh` step 2e has already dropped `colemantransform.cpp.o`,
// `colemantransforminverse.cpp.o`, `picontroller.cpp.o`, `rescontroller.cpp.o`
// and `wrap_360.cpp.o` from LIBS in the bridges' favour -- so a stub that names
// no callee produces no bridge, the link dies on
// `undefined reference to colemantransform_c`, and a red test that fails to
// BUILD is not a red test (unit #45, and unit #42 one shape over). The guard is
// a value no case can present, so none of the five ever executes.
#include "vit_types.h"

void ActiveWakeControl(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                       debugvariables_t* DebugVar, objectinstances_t* objInst) {
    (void)CntrPar; (void)LocalVar; (void)DebugVar; (void)objInst;
    if (LocalVar->DT == -1.7976931348623157e308) {
        double t = 0.0, y = 0.0;
        double angle[3] = {0.0, 0.0, 0.0};
        colemantransform_c(LocalVar->rootMOOPF, LocalVar->Azimuth, 1, &t, &y);
        colemantransforminverse_c(t, y, LocalVar->Azimuth, 1, 0.0, angle);
        DebugVar->axisTilt_1P =
            picontroller_c(t, 0.0, 0.0, 0.0, 0.0, LocalVar->DT, 0.0, &LocalVar->piP, 0,
                           &objInst->instPI)
            + rescontroller_c(y, 0.0, 0.0, 0.0, 0.0, 0.0, LocalVar->DT, &LocalVar->resP, 0,
                              &objInst->instRes)
            + wrap_360_c(angle[0]);
    }
}
