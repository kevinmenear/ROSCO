// VIT Translation Scaffold
// Function: SetpointSmoother
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
// Source MD5: 4a7edf120a16
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-28T12:00:16Z

#include "controlparameters_view_t.h"
#include "filterparameters_t.h"
#include "objectinstances_t.h"
#include "piparams_t.h"
#include "resparams_t.h"
#include "rlparams_t.h"
#include "we_t.h"
#include "localvariables_t.h"

extern "C" {

double lpfilter_c(double InputSignal, double DT, double CornerFreq,
                  filterparameters_t* FP, int iStatus, int reset, int* inst,
                  int has_InitialValue, double InitialValue);

void SetpointSmoother(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst) {

    // ------ Setpoint Smoothing ------
    if (CntrPar->SS_Mode == 1) {
        // Find setpoint shift amount
        double R_Total = LocalVar->PRC_R_Speed * LocalVar->PRC_R_Torque * LocalVar->PRC_R_Pitch;
        double DelOmega = ((LocalVar->BlPitchCMeas - LocalVar->PC_MinPit) / 0.524) * CntrPar->SS_VSGain
                        - ((CntrPar->VS_RtPwr * R_Total - LocalVar->VS_LastGenPwr)) / CntrPar->VS_RtPwr * CntrPar->SS_PCGain;
        DelOmega = DelOmega * CntrPar->PC_RefSpd;
        // Filter
        LocalVar->SS_DelOmegaF = lpfilter_c(DelOmega, LocalVar->DT, CntrPar->F_SSCornerFreq,
                                             &LocalVar->FP, LocalVar->iStatus,
                                             (LocalVar->restart != 0) ? 1 : 0,
                                             &objInst->instLPF, 0, 0.0);
    } else {
        LocalVar->SS_DelOmegaF = 0; // No setpoint smoothing
    }
}

} // extern "C"
