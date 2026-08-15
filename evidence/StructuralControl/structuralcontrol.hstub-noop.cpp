// RED-TEST STUB, not a translation. `StructuralControl` as a NO-OP.
//
// The whole unit does nothing: no mode test, no step, no open-loop lookup, no
// copy to avrSWAP, no error-message prefix. Every output the harness compares
// must therefore come back as the caller supplied it, and every case in which
// the reference writes ANY of them has to fail.
//
// What the number this produces is FOR: it is the count the green in
// harness/StructuralControl.json is a statement about. A corpus of 1227 cases
// that the no-op passes is a corpus that never reaches the unit; the difference
// between 1227 and the failure count here is the number of cases in which the
// reference leaves every compared output alone -- and for THIS unit that
// difference is expected to be large rather than zero, because the whole body
// is guarded: `StC_Mode` outside {1,2} skips the first block, `StC_Group_N = 0`
// skips the copy loop, and `aviFAIL >= 0` skips the message.
//
// NO CALLEE CALL IS KEPT, and that is a property of THIS tree rather than a
// general rule. `harness.sh` step 2e drops a generated callee bridge when the
// Fortran callee is already an integration wrapper -- which `interp1d` is here,
// so `structuralcontrol_callees.f90` carries ZERO bridges and
// `interp1d.cpp.o` is in LIBS instead. There is therefore no bridge whose
// retention depends on this file mentioning the callee, and the link does not
// need the discarded call that unit #39's `ResController` stub had to carry.

#include "vit_types.h"

void StructuralControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                       localvariables_view_t* LocalVar, objectinstances_t* objInst,
                       errorvariables_view_t* ErrVar) {
    (void)avrSWAP;
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
    (void)ErrVar;
}
