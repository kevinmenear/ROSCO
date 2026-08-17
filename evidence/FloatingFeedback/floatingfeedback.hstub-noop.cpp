// RED-TEST STUB, not a translation. `FloatingFeedback` as a NO-OP.
//
// The whole unit does nothing: no `interp1d`, no `PIController`, no mode test.
// It returns 0.0 and writes none of its three outputs.
//
// WHAT THE NUMBER THIS PRODUCES IS FOR: it is the count the green in
// harness/FloatingFeedback.json is a statement about. A corpus this no-op
// PASSES is a corpus that never reaches the unit.
//
// THE PREDICTION, WRITTEN BEFORE THE RUN: EVERY CASE FAILS. Unit #50's stub
// could name a pass set in advance -- `IF (Flp_Mode > 0)` guards its whole body
// and every case below it writes nothing. This unit has no such guard. It has
// three outputs and two of them are written on EVERY invocation whatever
// `Fl_Mode` holds:
//
//     LocalVar%Kp_Float   <- interp1d, unconditional
//     LocalVar%piP        <- two PIController calls, unconditional
//     objInst%instPI      <- post-incremented TWICE, unconditional
//
// so there is no configuration in which the reference leaves the state alone.
// If this run reports ANY pass, that is a finding about what the harness
// compares and not about the stub -- see the out-parameter list the run prints.
//
// THE TWO CALLEE CALLS ARE DELIBERATE AND ARE NOT DEAD CODE. Unit #45 measured
// that `vit/test_validate.generate_callee_bridges` decides which bridges to
// emit by running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the generated
// Makefile compiles and links `<stem>_callees.o` only when at least one bridge
// was emitted. On the CLEAN tree `scripts/harness.sh` step 2e keeps every
// bridge and drops the corresponding `<callee>.cpp.o` from LIBS, so a stub that
// mentions no callee changes the harness's own LINK -- and a red test that
// fails to BUILD is not a red test (unit #42). Both of this unit's callees --
// `interp1d` and `PIController` -- are themselves integrated, so both are in
// that trade.
//
// The guard is unsatisfiable on this corpus and on any corpus: `objInst` is
// always a real object here, so `objInst == nullptr` is never true, and the
// calls never execute. Their only job is to be TEXT the bridge generator can
// see.
//
// DO NOT DELETE THEM AS DEAD CODE.

#include "vit_types.h"

double FloatingFeedback(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, errorvariables_view_t* ErrVar) {
    if (objInst == nullptr) {
        // Unreachable. See the header: this exists so that the two callee
        // bridges are emitted and the harness links.
        LocalVar->Kp_Float = interp1d_c(CntrPar->Fl_U, CntrPar->n_Fl_U,
                                        CntrPar->Fl_Kp, CntrPar->n_Fl_Kp,
                                        LocalVar->WE_Vw_F, ErrVar);
        return picontroller_c(LocalVar->FA_AccF, 0.0, 1.0, -100.0, 100.0,
                              LocalVar->DT, 0.0, &LocalVar->piP,
                              LocalVar->restart ? 1 : 0, &objInst->instPI);
    }
    (void)LocalVar;
    (void)CntrPar;
    (void)objInst;
    (void)ErrVar;
    return 0.0;
}
