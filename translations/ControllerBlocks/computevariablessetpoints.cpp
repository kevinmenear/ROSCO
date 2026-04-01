// VIT Translation Scaffold
// Function: ComputeVariablesSetpoints
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
// Source MD5: bf8ac8a361c3
// VIT: 0.1.0
// Status: unverified

// Generated: 2026-03-28T12:00:17Z

#include "controlparameters_view_t.h"
#include "debugvariables_t.h"
#include "errorvariables_t.h"
#include "filterparameters_t.h"
#include "objectinstances_t.h"
#include "piparams_t.h"
#include "resparams_t.h"
#include "rlparams_t.h"
#include "we_t.h"
#include "localvariables_t.h"

#include <cmath>
#include "rosco_constants.h"

extern "C" {

double interp1d_c(double* xData, int n_xData, double* yData, int n_yData,
                  double xq, errorvariables_t* ErrVar);
double lpfilter_c(double InputSignal, double DT, double CornerFreq,
                  filterparameters_t* FP, int iStatus, int reset, int* inst,
                  int has_InitialValue, double InitialValue);
double saturate_c(double val, double minval, double maxval);
void refspeedexclusion_c(localvariables_t* LocalVar, controlparameters_view_t* CntrPar,
                         objectinstances_t* objInst, debugvariables_t* DebugVar);

void ComputeVariablesSetpoints(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {

    // Change pitch reference speed
    LocalVar->PC_RefSpd_PRC = CntrPar->PC_RefSpd * LocalVar->PRC_R_Speed;

    // Lookup table for speed setpoint (PRC_Mode 1)
    if (CntrPar->PRC_Mode == 1) {
        LocalVar->PRC_WSE_F = lpfilter_c(LocalVar->WE_Vw, LocalVar->DT, CntrPar->PRC_LPF_Freq,
                                          &LocalVar->FP, LocalVar->iStatus,
                                          (LocalVar->restart != 0) ? 1 : 0,
                                          &objInst->instLPF, 0, 0.0);
        LocalVar->PC_RefSpd_PRC = interp1d_c(CntrPar->PRC_WindSpeeds, CntrPar->n_PRC_WindSpeeds,
                                             CntrPar->PRC_GenSpeeds, CntrPar->n_PRC_GenSpeeds,
                                             LocalVar->PRC_WSE_F, ErrVar);
    }

    // Implement setpoint smoothing
    if (LocalVar->SS_DelOmegaF < 0) {
        LocalVar->PC_RefSpd_SS = LocalVar->PC_RefSpd_PRC - LocalVar->SS_DelOmegaF;
    } else {
        LocalVar->PC_RefSpd_SS = LocalVar->PC_RefSpd_PRC;
    }

    // Compute error for pitch controller
    LocalVar->PC_RefSpd = LocalVar->PC_RefSpd_SS;
    LocalVar->PC_SpdErr = LocalVar->PC_RefSpd - LocalVar->GenSpeedF;
    LocalVar->PC_PwrErr = CntrPar->VS_RtPwr - LocalVar->VS_GenPwr;

    // ----- Torque controller reference errors -----
    if (CntrPar->VS_ControlMode == VS_Mode_WSE_TSR) {
        LocalVar->VS_RefSpd_TSR = (CntrPar->VS_TSRopt * LocalVar->WE_Vw / CntrPar->WE_BladeRadius) * CntrPar->WE_GearboxRatio;

    } else if (CntrPar->VS_ControlMode == VS_Mode_Power_TSR) {
        double max_pwr = LocalVar->VS_GenPwr > 0.0 ? LocalVar->VS_GenPwr : 0.0;
        LocalVar->VS_RefSpd_TSR = pow(max_pwr / (CntrPar->VS_GenEff / 100.0) / CntrPar->VS_Rgn2K, 1.0 / 3.0);

    } else if (CntrPar->VS_ControlMode == VS_Mode_Torque_TSR) {
        double max_tq = LocalVar->GenTq > 0.0 ? LocalVar->GenTq : 0.0;
        LocalVar->VS_RefSpd_TSR = pow(max_tq / CntrPar->VS_Rgn2K, 1.0 / 2.0);

    } else {
        LocalVar->VS_RefSpd_TSR = CntrPar->VS_RefSpd;
    }

    // Region 3 FBP reference logic
    if (LocalVar->VS_RefSpd_TSR > CntrPar->VS_RefSpd) {
        if (CntrPar->VS_FBP == VS_FBP_WSE_Ref) {
            LocalVar->VS_RefSpd_TSR = interp1d_c(CntrPar->VS_FBP_U, CntrPar->n_VS_FBP_U,
                                                 CntrPar->VS_FBP_Omega, CntrPar->n_VS_FBP_Omega,
                                                 LocalVar->WE_Vw, ErrVar);
        } else if (CntrPar->VS_FBP == VS_FBP_Torque_Ref) {
            LocalVar->VS_RefSpd_TSR = interp1d_c(CntrPar->VS_FBP_Tau, CntrPar->n_VS_FBP_Tau,
                                                 CntrPar->VS_FBP_Omega, CntrPar->n_VS_FBP_Omega,
                                                 LocalVar->GenTq, ErrVar);
        }
    }

    // Change VS Ref speed based on R_Speed
    LocalVar->VS_RefSpd = LocalVar->VS_RefSpd_TSR * LocalVar->PRC_R_Speed;

    // Filter reference signal
    LocalVar->VS_RefSpd = lpfilter_c(LocalVar->VS_RefSpd_TSR, LocalVar->DT, CntrPar->F_VSRefSpdCornerFreq,
                                      &LocalVar->FP, LocalVar->iStatus,
                                      (LocalVar->restart != 0) ? 1 : 0,
                                      &objInst->instLPF, 0, 0.0);

    // Exclude reference speeds specified by user
    if (CntrPar->TRA_Mode > 0) {
        refspeedexclusion_c(LocalVar, CntrPar, objInst, DebugVar);
    }

    // Saturate torque reference speed below rated speed if using pitch control in Region 3
    if (CntrPar->VS_FBP == VS_FBP_Variable_Pitch) {
        LocalVar->VS_RefSpd = saturate_c(LocalVar->VS_RefSpd, CntrPar->VS_MinOMSpd, CntrPar->VS_RefSpd * LocalVar->PRC_R_Speed);
    }

    // Simple lookup table for generator speed (PRC_Mode 1)
    if (CntrPar->PRC_Mode == 1) {
        LocalVar->VS_RefSpd = interp1d_c(CntrPar->PRC_WindSpeeds, CntrPar->n_PRC_WindSpeeds,
                                         CntrPar->PRC_GenSpeeds, CntrPar->n_PRC_GenSpeeds,
                                         LocalVar->PRC_WSE_F, ErrVar);
    }

    // Implement setpoint smoothing
    if (LocalVar->SS_DelOmegaF > 0) {
        LocalVar->VS_RefSpd = LocalVar->VS_RefSpd - LocalVar->SS_DelOmegaF;
    }

    // Force minimum rotor speed
    LocalVar->VS_RefSpd = LocalVar->VS_RefSpd > CntrPar->VS_MinOMSpd ? LocalVar->VS_RefSpd : CntrPar->VS_MinOMSpd;

    // Compute speed error from reference
    LocalVar->VS_SpdErr = LocalVar->VS_RefSpd - LocalVar->GenSpeedF;

    // Define transition region setpoint errors
    LocalVar->VS_SpdErrAr = LocalVar->VS_RefSpd - LocalVar->GenSpeedF;
    LocalVar->VS_SpdErrBr = CntrPar->VS_MinOMSpd - LocalVar->GenSpeedF;

    // Region 3 minimum pitch angle for state machine
    LocalVar->VS_Rgn3Pitch = LocalVar->PC_MinPit + CntrPar->PC_Switch;

    // Debug Vars
    DebugVar->VS_RefSpd = LocalVar->VS_RefSpd;
    DebugVar->PC_RefSpd = LocalVar->PC_RefSpd;
}

} // extern "C"
