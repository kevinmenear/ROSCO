// RED-TEST STUB, not a translation. `ForeAftDamping` as a NO-OP.
//
// The whole unit does nothing: no `PIController`, no loop, no write.
//
// WHAT THE NUMBER THIS PRODUCES IS FOR: it is the count the green in
// harness/ForeAftDamping.json is a statement about. A corpus this no-op PASSES
// is a corpus that never reaches the unit.
//
// THE PREDICTION, WRITTEN BEFORE THE RUN: EVERY CASE FAILS, and this unit can
// say so more flatly than any before it because it has no branch of any kind.
// Unit #50's stub could name a pass set in advance (`IF (Flp_Mode > 0)` guards
// its whole body); unit #51's could not, and predicted the whole corpus. This
// one has three outputs and two of them are written on EVERY invocation
// whatever any input holds:
//
//     LocalVar%FA_AccHPFI   <- the PIController result, unconditional
//     LocalVar%piP          <- written by that call, unconditional
//     objInst%instPI        <- post-incremented by it, unconditional
//
// The third, `LocalVar%FA_PitCom`, is written `NumBl` times and is the only
// output a corpus case can leave alone (at `NumBl == 0`) -- but a case that
// does so still moves the other two. So the pass set should be EMPTY: the
// expected result is `failed == checked`, and any pass at all is a finding
// about what the harness compares rather than about the stub. Read the run's
// printed out-parameter list before explaining one.
//
// THE CALLEE CALL IS DELIBERATE AND IS NOT DEAD CODE. Unit #45 measured that
// `vit/test_validate.generate_callee_bridges` decides which bridges to emit by
// running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the generated
// Makefile compiles and links `<stem>_callees.o` only when at least one bridge
// was emitted. On the CLEAN tree `scripts/harness.sh` step 2e keeps every
// bridge and drops the corresponding `<callee>.cpp.o` from LIBS, so a stub that
// mentions no callee changes the harness's own LINK -- and a red test that
// fails to BUILD is not a red test (unit #42). This unit's one callee,
// `PIController`, is itself integrated, so it is in that trade.
//
// The guard is unsatisfiable on this corpus and on any corpus: `objInst` is
// always a real object here, so `objInst == nullptr` is never true, and the
// call never executes. Its only job is to be TEXT the bridge generator can see.
//
// DO NOT DELETE IT AS DEAD CODE.

#include "vit_types.h"

void ForeAftDamping(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar, objectinstances_t* objInst) {
    if (objInst == nullptr) {
        // Unreachable. See the header: this exists so that the PIController
        // bridge is emitted and the harness links.
        LocalVar->FA_AccHPFI = picontroller_c(LocalVar->FA_AccHPF, 0.0,
                                              CntrPar->FA_KI,
                                              -CntrPar->FA_IntSat,
                                              CntrPar->FA_IntSat,
                                              LocalVar->DT, 0.0,
                                              &LocalVar->piP,
                                              LocalVar->restart ? 1 : 0,
                                              &objInst->instPI);
    }
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
}
