// RED-TEST STUB, not a translation. `CableControl` as a NO-OP.
//
// The whole unit does nothing: no mode test, no step, no open-loop lookup, no
// filter, no integrator, no copy to avrSWAP, no error-message prefix. Every
// output the harness compares must therefore come back as the caller supplied
// it, and every case in which the reference writes ANY of them has to fail.
//
// WHAT THE NUMBER THIS PRODUCES IS FOR: it is the count the green in
// harness/CableControl.json is a statement about. A corpus this no-op PASSES is
// a corpus that never reaches the unit. The difference between the corpus size
// and the failure count is the number of cases in which the reference leaves
// every compared output alone -- and for this unit that difference is expected
// to be small rather than large, unlike unit #43's: `CC_Group_N = 0` is the only
// draw that skips the filter/integrator loop, and that loop writes
// `CC_ActuatedDL` and `CC_ActuatedL` on EVERY case that runs it, whatever
// `CC_Mode` holds. Unit #43's body is guarded end to end; this one is not.
//
// THE THREE CALLEE CALLS ARE DELIBERATE AND ARE NOT DEAD CODE. Unit #45
// measured that `vit/test_validate.generate_callee_bridges` decides which
// bridges to emit by running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the
// generated Makefile compiles and links `<stem>_callees.o` only when at least
// one bridge was emitted. On the CLEAN tree `scripts/harness.sh` step 2e keeps
// every bridge and drops the corresponding `<callee>.cpp.o` from LIBS, so a stub
// that mentions no callee changes the harness's own LINK -- and a red test that
// fails to BUILD is not a red test (unit #42).
//
// The guard is unsatisfiable on this corpus and on any corpus: `ErrVar` is
// always a real object here, so `ErrVar == nullptr` is never true, and the calls
// never execute. Their only job is to be TEXT the bridge generator can see.
//
// DO NOT DELETE THEM AS DEAD CODE.

#include "vit_types.h"

void CableControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                  localvariables_view_t* LocalVar, objectinstances_t* objInst,
                  errorvariables_view_t* ErrVar) {
    if (ErrVar == nullptr) {
        // Unreachable. See the header: this exists so that the three callee
        // bridges are emitted and the harness links.
        LocalVar->CC_DesiredL[0] =
            interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                       CntrPar->OL_CableControl, CntrPar->n_OL_CableControl_cols,
                       LocalVar->Time, ErrVar);
        LocalVar->CC_ActuatedDL[0] = seclpfilter_vel_c(
            LocalVar->CC_DesiredL[0], LocalVar->DT, 1.0, 1.0, &LocalVar->FP,
            LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instSecLPFV,
            0, 0.0);
        LocalVar->CC_ActuatedL[0] = picontroller_c(
            LocalVar->CC_ActuatedDL[0], 0.0, 1.0, -1000.0, 1000.0, LocalVar->DT,
            LocalVar->CC_ActuatedDL[0], &LocalVar->piP,
            LocalVar->restart ? 1 : 0, &objInst->instPI);
    }
    (void)avrSWAP;
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
    (void)ErrVar;
}
