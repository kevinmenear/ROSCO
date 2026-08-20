// STUB -- NOT THE SHIPPED TRANSLATION. Unit #59 harness red test.
// THE WHOLE BODY DELETED. One ratelimit_c call is kept behind a guard no
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

    if (LocalVar->DT != LocalVar->DT && LocalVar->DT == 0.0) {
        LocalVar->VS_RefSpd_RL =
            ratelimit_c(0.0, 0.0, 0.0, 1.0, 0, &LocalVar->rlP, &objInst->instRL, 0, 0.0);
    }
}
