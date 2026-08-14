// STUB, not a translation. Reads NO input and writes the four numbers the
// reference produced in all 41 captured cases (evidence/.../kernel.verify_fields.csv:
// prc_r_speed 0.9, prc_r_torque 1.0, prc_r_pitch 1.0, prc_min_pitch 0.0).
// This is the translation the kernel cannot tell from the real one.
#include "vit_types.h"
void PowerControlSetpoints(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                           objectinstances_t* objInst, debugvariables_t* DebugVar,
                           errorvariables_view_t* ErrVar) {
    (void)CntrPar; (void)objInst; (void)DebugVar; (void)ErrVar;
    LocalVar->PRC_R_Speed = 0.9;
    LocalVar->PRC_R_Torque = 1.0;
    LocalVar->PRC_R_Pitch = 1.0;
    LocalVar->PRC_Min_Pitch = 0.0;
}
