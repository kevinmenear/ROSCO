// RED-TEST STUB, not a translation. `YawRateControl` as a NO-OP.
//
// The whole unit does nothing: no `Y_ControlMode` test, no wind-direction
// update, no filtering, no state machine, no write to `avrSWAP(48)`, no
// open-loop override, none of the five DebugVar stores. Every output the
// harness compares must therefore come back as the caller supplied it, and
// every case in which the reference writes ANY of them has to fail.
//
// What the number this produces is FOR: it is the count the green in
// harness/YawRateControl.json is a statement about. A corpus of 8036 cases
// that the no-op passes is a corpus that never reaches the unit; the difference
// between 8036 and the failure count here is the number of cases in which the
// reference leaves every compared output alone -- and for THIS unit that
// difference is expected to be substantial rather than zero, because the entire
// body sits behind `Y_ControlMode == 1` and the generator's own predicate knob
// puts that parameter at 0, 1 and 2.
//
// AND ONE THING ONLY THIS STUB CAN SAY. `INTEGER, SAVE :: YawState` is ambient
// state that no case sets: the harness advances it by calling the unit, so a
// no-op leaves the C++ side's static at 0 for the whole run while the Fortran
// side's SAVE walks the state machine. That divergence is part of what this
// count measures, and it is the reason the count is expected to be far larger
// than the number of cases in which `Y_ControlMode == 1`.
//
// NO CALLEE CALL IS KEPT, for the reason unit #43's stub states one directory
// over: `harness.sh` step 2e drops a generated callee bridge when the Fortran
// callee is already an integration wrapper, and all three of this unit's
// callees -- `wrap_180`, `LPFilter` and `interp1d` -- are integrated, so
// `yawratecontrol_callees.f90` carries zero bridges and the three `.cpp.o`
// files are in LIBS instead. There is no bridge whose retention depends on this
// file mentioning a callee, so unit #39's discarded call is not needed here.

#include "vit_types.h"

void YawRateControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                    localvariables_view_t* LocalVar, objectinstances_t* objInst,
                    debugvariables_t* DebugVar, errorvariables_view_t* ErrVar) {
    (void)avrSWAP;
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
    (void)DebugVar;
    (void)ErrVar;
}
