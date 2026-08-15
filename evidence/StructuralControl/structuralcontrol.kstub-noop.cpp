// KERNEL RED-TEST STUB, not a translation. `StructuralControl` as a NO-OP.
//
// `vit verify` DECLINED to construct its own red test for this unit and printed
// NON_DISCRIMINATING -- the fourth unit running, and for the same stated reason:
// every argument arrives behind a pointer or inside a derived type, so there is
// no by-value floating-point input it can perturb without changing what BOTH
// implementations compute. The kernel's discriminating power therefore has to be
// measured by hand or not at all, and these stubs are the measurement.
//
// PREDICTED BEFORE IT WAS RUN: this stub fails EXACTLY ONE of the 62 cases,
// `StructuralControl.0.0.20002`. The argument is vit.yaml's own, written before
// the extraction: `avrSWAP` is INOUT and persists across calls in the driver,
// and `LocalVar%StC_Input` likewise, so from invocation 20,003 onward the values
// this unit writes are ALREADY THERE on entry and a unit that does nothing
// reproduces them. Cases 1-20 and 19,995-20,001 are below the `Time > 500` step
// and the reference writes nothing either. 20,002 is the single invocation in
// the whole 23,999-call run at which `StC_Input` CHANGES value.

#include "vit_types.h"

void StructuralControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                       localvariables_view_t* LocalVar, objectinstances_t* objInst,
                       errorvariables_view_t* ErrVar) {
    (void)avrSWAP; (void)CntrPar; (void)LocalVar; (void)objInst; (void)ErrVar;
}
