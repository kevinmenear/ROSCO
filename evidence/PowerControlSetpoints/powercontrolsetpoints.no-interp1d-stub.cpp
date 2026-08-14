// STUB, not a translation. The real body with the PRC_Min_Pitch interp1d call
// DELETED -- the one statement in the PRC_Mode==2 arm that reads a table and
// calls this unit's only callee. Asks whether the kernel can see the callee at
// all, or whether the reference's own answer there is a constant the field
// would carry anyway.
#include "vit_types.h"
void PowerControlSetpoints(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                           objectinstances_t* objInst, debugvariables_t* DebugVar,
                           errorvariables_view_t* ErrVar) {
    (void)objInst; (void)DebugVar;
    if (CntrPar->PRC_Mode == 2) {
        if (CntrPar->PRC_Comm == 0) {
            LocalVar->PRC_R_Speed = CntrPar->PRC_R_Speed;
            LocalVar->PRC_R_Torque = CntrPar->PRC_R_Torque;
            LocalVar->PRC_R_Pitch = CntrPar->PRC_R_Pitch;
        } else if (CntrPar->PRC_Comm == 1) {
            if (CntrPar->Ind_R_Speed > 0) {
                LocalVar->PRC_R_Speed =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_R_Speed, CntrPar->n_OL_R_Speed,
                               LocalVar->OL_Index, ErrVar);
            } else {
                LocalVar->PRC_R_Speed = 1.0;
            }
            if (CntrPar->Ind_R_Torque > 0) {
                LocalVar->PRC_R_Torque =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_R_Torque, CntrPar->n_OL_R_Torque,
                               LocalVar->OL_Index, ErrVar);
            } else {
                LocalVar->PRC_R_Torque = 1.0;
            }
            if (CntrPar->Ind_R_Pitch > 0) {
                LocalVar->PRC_R_Pitch =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_R_Pitch, CntrPar->n_OL_R_Pitch,
                               LocalVar->OL_Index, ErrVar);
            } else {
                LocalVar->PRC_R_Pitch = 1.0;
            }
        } else if (CntrPar->PRC_Comm == 2) {
            LocalVar->PRC_R_Speed = LocalVar->ZMQ_R_Speed;
            LocalVar->PRC_R_Torque = LocalVar->ZMQ_R_Torque;
            LocalVar->PRC_R_Pitch = LocalVar->ZMQ_R_Pitch;
        }
        // the PRC_Min_Pitch interp1d call is deleted here
    } else {
        LocalVar->PRC_R_Speed = 1.0;
        LocalVar->PRC_R_Torque = 1.0;
        LocalVar->PRC_R_Pitch = 1.0;
        LocalVar->PRC_Min_Pitch = CntrPar->PC_FinePit;
    }
}
