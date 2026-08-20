// KERNEL RED TEST STUB for unit #60 VariableSpeedControl -- the no-op
//
// NOT A TRANSLATION. `vit verify` printed NON_DISCRIMINATING for this unit --
// "no by-value floating-point parameter and no floating-point result: every
// input arrives behind a pointer or inside a derived type" -- the same refusal
// it gave units #40 through #44. So the kernel's 60/60 has to be red-tested by
// hand or its discriminating power stays unmeasured.
//
// The WHOLE BODY is deleted. Every compared field this unit writes --
// vs_komega2_gentq, vs_constpwr_gentq, vs_maxtq, genartq, genbrtq, gentq,
// gentq_sd, vs_lastgentrq, vs_lastgenpwr, instpi, instrl -- keeps whatever
// the captured in-state holds.
//
// EXPECT: 60 of 60 FAIL. `instpi` alone settles it: the reference issues two
// PIController calls on every one of these 60 cases (VS_ControlMode == 1 in
// all of them, measured in verify_fields.csv) and each post-increments the
// counter, so the reference leaves instpi + 2 and this stub leaves instpi.

#include "vit_types.h"

void VariableSpeedControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                          localvariables_view_t* LocalVar, objectinstances_t* objInst,
                          errorvariables_view_t* ErrVar) {
    (void)avrSWAP;
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
    (void)ErrVar;
}
