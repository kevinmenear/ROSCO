// KERNEL RED-TEST STUB, not a translation. `Startup` as a NO-OP.
//
// The whole unit does nothing: no filter, no stage machine, no setpoint. Every
// output the harness compares must therefore come back as the caller supplied
// it, and every case in which the reference writes ANY of them has to fail.
//
// What the number this produces is FOR: it is the count the green in
// evidence/Startup/kernel.verify_fields.csv is a statement about. `vit verify`
// declined to build a red test on this kernel and printed NON_DISCRIMINATING,
// so 83/83 IDENTICAL says nothing on its own until some deliberate change is
// shown to break it. This is that change, in its crudest form.
//
// Case 1 is the one case with `iStatus == 0`, where the reference's first act
// is `SU_Stage = -1`; cases 2-200 sit in stage 1, 201-1000 in stage 2 and
// 1001-1800 in stage 3, and 1801 is the single invocation that completes
// startup. A no-op has to disagree on all of them.
//
// NO CALLEE CALL IS KEPT. The kernel links `vit_kernel_callees.h`'s
// declarations against the kernel's own Fortran, which defines `lpfilter_c` and
// `sigma_c` whether or not this file mentions them, so the bridge-retention
// problem unit #39's `ResController` stub had is not this instrument's.

#include "vit_types.h"

void Startup(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
             objectinstances_t* objInst, errorvariables_view_t* ErrVar) {
    (void)LocalVar;
    (void)CntrPar;
    (void)objInst;
    (void)ErrVar;
}
