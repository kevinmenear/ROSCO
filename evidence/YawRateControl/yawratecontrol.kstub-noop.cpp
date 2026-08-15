// KERNEL RED-TEST STUB, not a translation. `YawRateControl` as a NO-OP.
//
// Asks: how much of what this unit writes can the 104-case kernel see at all?
//
// Every compared field must come back as the case supplied it. The kernel
// compares ~433 field names per case -- CntrPar, LocalVar, objInst, DebugVar
// and ErrVar in full -- so what this stub leaves untouched includes
// LocalVar%WindDir, objInst%instLPF, LocalVar%FP's six filter arrays and the
// five DebugVar stores. Scenario 7 sets Y_ControlMode = 1, so the body runs on
// every one of the 104 captured invocations.

#include "vit_types.h"

void YawRateControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                    localvariables_view_t* LocalVar, objectinstances_t* objInst,
                    debugvariables_t* DebugVar, errorvariables_view_t* ErrVar) {
    (void)avrSWAP; (void)CntrPar; (void)LocalVar;
    (void)objInst; (void)DebugVar; (void)ErrVar;
}
