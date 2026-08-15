// RED-TEST STUB, not a translation. `Startup` as a NO-OP.
//
// The whole unit does nothing: no filter, no stage machine, no setpoint. Every
// output the harness compares must therefore come back as the caller supplied
// it, and every case in which the reference writes ANY of them has to fail.
//
// What the number this produces is FOR: it is the count the green in
// harness/Startup.json is a statement about. A corpus of 7103 cases that the
// no-op passes is a corpus that never reaches the unit; the difference between
// 7103 and the failure count here is the number of cases in which the reference
// leaves every compared output alone.
//
// NO CALLEE CALL IS KEPT, and that is a property of THIS tree rather than a
// general rule. `harness.sh` step 2e drops a generated callee bridge when the
// Fortran callee is already an integration wrapper -- which `LPFilter` and
// `sigma` both are here -- and keeps `lpfilter.cpp.o` / `sigma.cpp.o` in LIBS
// instead. So there is no bridge whose retention depends on this file
// mentioning the callee, and the link does not need the discarded call that
// unit #39's `ResController` stub had to carry.

#include "vit_types.h"

void Startup(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
             objectinstances_t* objInst, errorvariables_view_t* ErrVar) {
    (void)LocalVar;
    (void)CntrPar;
    (void)objInst;
    (void)ErrVar;
}
