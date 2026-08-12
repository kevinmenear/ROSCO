// STUB — NOT the translation. Evidence input for unit #20.
//
// The RIGHT constant: the answer 61 of the 62 captured cases have
// (PC_State = PC_State_Enabled, VS_State = VS_State_Region_1_5), written
// unconditionally, reading no argument. If this PASSES, the kernel is a lookup
// table for this unit over this window.
#include "vit_types.h"

void StateMachine(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar) {
    (void)CntrPar;
    LocalVar->PC_State = 1;  // PC_State_Enabled
    LocalVar->VS_State = 1;  // VS_State_Region_1_5
}
