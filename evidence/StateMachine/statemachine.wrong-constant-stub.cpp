// STUB — NOT the translation. Evidence input for unit #20.
//
// The WRONG constant: PC_State = PC_State_Disabled and VS_State = VS_State_PI
// (7), a value no branch of this unit can produce at all. If this FAILS, the
// comparison is alive on the VALUE and not merely on presence. This is the
// liveness test whenever the arguments alias — the no-op cannot be one here.
#include "vit_types.h"

void StateMachine(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar) {
    (void)CntrPar;
    LocalVar->PC_State = 0;  // PC_State_Disabled
    LocalVar->VS_State = 7;  // VS_State_PI — produced by no branch of this unit
}
