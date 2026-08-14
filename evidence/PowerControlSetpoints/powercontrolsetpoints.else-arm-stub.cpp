// STUB, not a translation. The OUTER predicate inverted: the ELSE arm is taken
// unconditionally, so PRC_Mode == 2 is ignored. This is the coarsest possible
// wrong control flow in this unit.
#include "vit_types.h"
void PowerControlSetpoints(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                           objectinstances_t* objInst, debugvariables_t* DebugVar,
                           errorvariables_view_t* ErrVar) {
    (void)objInst; (void)DebugVar; (void)ErrVar;
    LocalVar->PRC_R_Speed = 1.0;
    LocalVar->PRC_R_Torque = 1.0;
    LocalVar->PRC_R_Pitch = 1.0;
    LocalVar->PRC_Min_Pitch = CntrPar->PC_FinePit;
}
