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
    // ARM CENSUS probe A -- the `SS_Mode == 1` arm deleted, the ELSE arm intact.
    // Fails exactly the cases that reach the IF arm AND let it move an output.
    if (CntrPar->SS_Mode == 1) {
        static volatile int never = 0;   // links `lpfilter_c`; executes 0 times
        if (never) {
            (void)lpfilter_c(0.0, 0.0, 0.0, &LocalVar->FP, 0, 0, &objInst->instLPF, 0, 0.0);
        }
    } else {
        LocalVar->SS_DelOmegaF = 0;
    }
}
