// VIT Translation Scaffold
// Function: SetpointSmoother
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 00345d404b2f
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T03:27:49Z
//
// Unit #40. `ControllerBlocks.f90:490-517` in the clean source.
//
// The whole unit is two arms and four statements, and both arms write exactly
// one compared field, `LocalVar%SS_DelOmegaF`.
//
// THE `/` AND `*` IN THE SECOND TERM ASSOCIATE LEFT TO RIGHT AND THE ALGEBRA
// IS NOT THE SHAPE. The reference writes
//
//     ... - ((VS_RtPwr * R_Total - VS_LastGenPwr))/VS_RtPwr * SS_PCGain
//
// which is `((num / VS_RtPwr) * SS_PCGain)`, a divide and then a multiply --
// NOT `num * SS_PCGain / VS_RtPwr` and not `num / (VS_RtPwr / SS_PCGain)`.
// Transcribed here in that order, parenthesised so nothing later re-associates
// it. `0.524` divides before its multiply for the same reason.
//
// `LocalVar%FP` is a nested derived type held BY VALUE inside the view struct,
// so its address is taken here and the wrapper must copy it back: this unit
// integrates with `--reverse-copy`. It is the callee, not this unit, that
// writes it -- the same shape unit #38 recorded.

#include "vit_types.h"

void SetpointSmoother(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst) {
    // RED TEST -- the unit as a no-op. Not a translation.
    //
    // THE CALL IS HERE FOR THE LINKER AND IT MUST NOT RUN. Unit #39 records why
    // it is here: `harness.sh` reads the translation for calls, and on seeing
    // `lpfilter_c` it keeps `lpfilter.cpp.o` in LIBS ("dropping the lpfilter_c
    // bridge -- lpfilter.cpp.o already defines it"). A stub that calls nothing
    // keeps nothing, and every other translated .cpp calling `lpfilter_c` then
    // fails to link -- a build failure that reads like a broken harness.
    //
    // The FIRST version of this stub made the call live, with constant scalars
    // and the real `&objInst->instLPF`. It BUILT AND THEN CRASHED, and the
    // harness reported "harness produced no JSON" -- not a red test, a dead
    // process. The reference calls LPFilter only on the `SS_Mode == 1` arm; on
    // the other arm it never passes `instLPF` to a filter at all, so on those
    // cases the stub was handing the callee an instance index the reference
    // never hands it. `never` is `static volatile`, so the compiler cannot fold
    // the branch away and the symbol reference survives to the link, while the
    // call itself executes zero times in 5599 cases.
    (void)CntrPar;
    static volatile int never = 0;
    if (never) {
        (void)lpfilter_c(0.0, 0.0, 0.0, &LocalVar->FP, 0, 0, &objInst->instLPF, 0, 0.0);
    }
}
