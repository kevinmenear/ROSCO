// STUB -- NOT THE SHIPPED TRANSLATION. Unit #59, and it is a PREDICTOR rather
// than a red test of its own.
//
// THE WHOLE BODY DELETED EXCEPT `objInst%instRL`, WHICH IS ADVANCED BY HAND.
// `instRL` is the ONE output of this unit that does not live inside `LocalVar`:
// it crosses as a plain `objectinstances_t*` and the integrated wrapper passes
// it by `C_LOC` with no view and no copy-back. Every other output -- FA_Hist,
// TRA_LastRefSpd, VS_RefSpd_TRA, VS_RefSpd_RL, VS_RefSpd and the nested
// `rlP%LastSignal` -- reaches the caller ONLY through
// `vit_copy_scalars_to_localvariables`.
//
// So the set of cases this stub fails is exactly the set of cases in which the
// unit changes something inside `LocalVar` -- which is exactly the set the
// POST-INTEGRATION wrapper red test (that copy-back deleted) must fail. One
// pre-integration run, 15 seconds, and the post-integration prediction stops
// being a bracket.
//
// THE PLAIN BODY DELETED. One ratelimit_c call is kept behind a guard no
// admissible case satisfies, because `vit/test_validate.generate_callee_bridges`
// finds callees by a REGEX OVER THIS FILE: a stub that calls nothing gets no
// `refspeedexclusion_callees.o` generated, and the link then dies on symbols the
// OTHER integrated units need (unit #45). Do not delete it as dead code.
//
// Built by hand from translations/ControllerBlocks/refspeedexclusion.cpp at
// commit 40d869e3; every comment of the original is dropped so that what is
// CUT is visible at a glance.

#include "vit_types.h"

void RefSpeedExclusion(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar) {
    (void)CntrPar; (void)DebugVar;

    // The reference advances the instance counter exactly once per call, inside
    // `ratelimit`, on both arms. Reproduced here and nowhere else.
    objInst->instRL = objInst->instRL + 1;

    if (LocalVar->DT != LocalVar->DT && LocalVar->DT == 0.0) {
        LocalVar->VS_RefSpd_RL =
            ratelimit_c(0.0, 0.0, 0.0, 1.0, 0, &LocalVar->rlP, &objInst->instRL, 0, 0.0);
    }
}
