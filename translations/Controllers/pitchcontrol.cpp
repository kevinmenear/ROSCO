// VIT Translation
// Function: PitchControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
// Source MD5: 9c79c278f64a
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-24T17:41:38Z

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

#include <algorithm>
#include <cstring>
#include <cstdio>

#include "rosco_constants.h"

// Callee entry points
extern "C" {
    double picontroller_c(double error, double kp, double ki,
                          double minValue, double maxValue, double DT,
                          double I0, piparams_t* piP, int reset, int* inst);
    double interp1d_c(double* xData, int n_xData, double* yData, int n_yData,
                      double xq, errorvariables_t* ErrVar);
    double saturate_c(double val, double minval, double maxval);
    double ratelimit_c(double inputSignal, double minRate, double maxRate, double DT,
                       int reset, rlparams_t* rlP, int* inst,
                       int has_ResetValue, double ResetValue);
    void ipc_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar,
               objectinstances_t* objInst, debugvariables_t* DebugVar,
               errorvariables_t* ErrVar);
    void foreaftdamping_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar,
                          objectinstances_t* objInst);
    double pitchsaturation_c(localvariables_t* LocalVar, controlparameters_view_t* CntrPar,
                             objectinstances_t* objInst, debugvariables_t* DebugVar,
                             errorvariables_t* ErrVar);
    double floatingfeedback_c(localvariables_t* LocalVar, controlparameters_view_t* CntrPar,
                              objectinstances_t* objInst, errorvariables_t* ErrVar);
    void activewakecontrol_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar,
                             debugvariables_t* DebugVar, objectinstances_t* objInst);
    double lpfilter_c(double InputSignal, double DT, double CornerFreq,
                      filterparameters_t* FP, int iStatus, int reset, int* inst,
                      int has_InitialValue, double InitialValue);
    double seclpfilter_c(double InputSignal, double DT, double CornerFreq,
                         double Damp, filterparameters_t* FP, int iStatus,
                         int reset, int* inst,
                         int has_InitialValue, double InitialValue);
}

void PitchControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {
    // PitchControl: master blade pitch controller
    // Orchestrates collective pitch PI, IPC, tower damping, floating feedback,
    // pitch saturation, AWC, shutdown, actuator model, and fault handling.

    // Load PC State
    if (LocalVar->PC_State == PC_State_Enabled) {
        LocalVar->PC_MaxPit = CntrPar->PC_MaxPit;
    } else {
        LocalVar->PC_MaxPit = CntrPar->PC_FinePit;
    }

    // Hold blade pitch at last value in pre-startup mode
    if ((CntrPar->SU_Mode > 0) && (LocalVar->SU_Stage == -1)) {
        LocalVar->PC_MaxPit = LocalVar->BlPitchCMeas;
        LocalVar->PC_MinPit = LocalVar->BlPitchCMeas;
    }

    // Gain scheduling via interpolation
    LocalVar->PC_KP = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                  CntrPar->PC_GS_KP, CntrPar->n_PC_GS_KP,
                                  LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_KI = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                  CntrPar->PC_GS_KI, CntrPar->n_PC_GS_KI,
                                  LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_KD = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                  CntrPar->PC_GS_KD, CntrPar->n_PC_GS_KD,
                                  LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_TF = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                  CntrPar->PC_GS_TF, CntrPar->n_PC_GS_TF,
                                  LocalVar->BlPitchCMeasF, ErrVar);

    // Collective pitch PI controller
    LocalVar->PC_PitComT = picontroller_c(
        LocalVar->PC_SpdErr, LocalVar->PC_KP, LocalVar->PC_KI,
        LocalVar->PC_MinPit, LocalVar->PC_MaxPit,
        LocalVar->DT, LocalVar->BlPitch[0],
        &LocalVar->piP, (LocalVar->restart != 0), &objInst->instPI);
    DebugVar->PC_PICommand = LocalVar->PC_PitComT;

    // Individual pitch control
    if ((CntrPar->IPC_ControlMode >= 1) || (CntrPar->Y_ControlMode == 2)) {
        ipc_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar);
    } else {
        LocalVar->IPC_PitComF[0] = 0.0;
        LocalVar->IPC_PitComF[1] = 0.0;
        LocalVar->IPC_PitComF[2] = 0.0;
    }

    // Tower fore-aft damping
    if (CntrPar->TD_Mode > 0) {
        foreaftdamping_c(CntrPar, LocalVar, objInst);
    } else {
        LocalVar->FA_PitCom[0] = 0.0;
        LocalVar->FA_PitCom[1] = 0.0;
        LocalVar->FA_PitCom[2] = 0.0;
    }

    // Pitch saturation
    if (CntrPar->PS_Mode > 0) {
        LocalVar->PC_MinPit = pitchsaturation_c(LocalVar, CntrPar, objInst, DebugVar, ErrVar);
        LocalVar->PC_MinPit = std::max(LocalVar->PC_MinPit, CntrPar->PC_FinePit);
    } else {
        LocalVar->PC_MinPit = CntrPar->PC_FinePit;
    }
    DebugVar->PC_MinPit = LocalVar->PC_MinPit;

    // Floating feedback
    if (CntrPar->Fl_Mode > 0) {
        LocalVar->Fl_PitCom = floatingfeedback_c(LocalVar, CntrPar, objInst, ErrVar);
        DebugVar->Fl_PitCom = LocalVar->Fl_PitCom;
        LocalVar->PC_PitComT += LocalVar->Fl_PitCom;
    }

    // Saturate collective pitch
    LocalVar->PC_PitComT = saturate_c(LocalVar->PC_PitComT, LocalVar->PC_MinPit, CntrPar->PC_MaxPit);
    LocalVar->PC_PitComT = ratelimit_c(LocalVar->PC_PitComT, CntrPar->PC_MinRat, CntrPar->PC_MaxRat,
                                        LocalVar->DT, (LocalVar->restart != 0),
                                        &LocalVar->rlP, &objInst->instRL,
                                        1, LocalVar->BlPitchCMeas);
    LocalVar->PC_PitComT_Last = LocalVar->PC_PitComT;

    // Combine and saturate individual pitch commands
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->PitCom[K] = LocalVar->PC_PitComT + LocalVar->FA_PitCom[K];
        LocalVar->PitCom[K] = saturate_c(LocalVar->PitCom[K], LocalVar->PC_MinPit, CntrPar->PC_MaxPit);
        LocalVar->PitCom[K] += LocalVar->IPC_PitComF[K];

        // Hard IPC saturation by peak shaving limit
        if (CntrPar->IPC_SatMode == 1) {
            LocalVar->PitCom[K] = saturate_c(LocalVar->PitCom[K], LocalVar->PC_MinPit, CntrPar->PC_MaxPit);
        }

        // ZeroMQ pitch offset
        LocalVar->PitCom[K] += LocalVar->ZMQ_PitOffset[K];

        // Rate limit per blade
        LocalVar->PitCom[K] = ratelimit_c(LocalVar->PitCom[K], CntrPar->PC_MinRat, CntrPar->PC_MaxRat,
                                           LocalVar->DT, (LocalVar->restart != 0),
                                           &LocalVar->rlP, &objInst->instRL,
                                           1, LocalVar->BlPitch[K]);
    }

    // Open loop pitch control
    if (CntrPar->OL_Mode > 0) {
        if (LocalVar->Time >= CntrPar->OL_Breakpoints[0]) {
            if (CntrPar->Ind_BldPitch[0] > 0) {
                LocalVar->PitCom[0] = interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                                                   CntrPar->OL_BldPitch1, CntrPar->n_OL_BldPitch1,
                                                   LocalVar->OL_Index, ErrVar);
            }
            if (CntrPar->Ind_BldPitch[1] > 0) {
                LocalVar->PitCom[1] = interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                                                   CntrPar->OL_BldPitch2, CntrPar->n_OL_BldPitch2,
                                                   LocalVar->OL_Index, ErrVar);
            }
            if (CntrPar->Ind_BldPitch[2] > 0) {
                LocalVar->PitCom[2] = interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                                                   CntrPar->OL_BldPitch3, CntrPar->n_OL_BldPitch3,
                                                   LocalVar->OL_Index, ErrVar);
            }
        }
    }

    // Active wake control
    if (CntrPar->AWC_Mode > 0) {
        activewakecontrol_c(CntrPar, LocalVar, DebugVar, objInst);
    }

    // Shutdown
    if (LocalVar->SD_Trigger == 0) {
        LocalVar->PitCom_SD[0] = LocalVar->PitCom[0];
        LocalVar->PitCom_SD[1] = LocalVar->PitCom[1];
        LocalVar->PitCom_SD[2] = LocalVar->PitCom[2];
    } else {
        if (CntrPar->SD_Method == 1 || CntrPar->SD_Method == 2) {
            for (int K = 0; K < LocalVar->NumBl; K++) {
                LocalVar->PitCom_SD[K] += LocalVar->SD_MaxPitchRate * LocalVar->DT;
            }
        }
        LocalVar->PitCom[0] = LocalVar->PitCom_SD[0];
        LocalVar->PitCom[1] = LocalVar->PitCom_SD[1];
        LocalVar->PitCom[2] = LocalVar->PitCom_SD[2];
    }

    // Pitch actuator model
    for (int K = 0; K < LocalVar->NumBl; K++) {
        if (CntrPar->PA_Mode > 0) {
            if (CntrPar->PA_Mode == 1) {
                LocalVar->PitComAct[K] = lpfilter_c(
                    LocalVar->PitCom[K], LocalVar->DT, CntrPar->PA_CornerFreq,
                    &LocalVar->FP, LocalVar->iStatus,
                    (LocalVar->restart != 0), &objInst->instLPF,
                    0, 0.0);
            } else if (CntrPar->PA_Mode == 2) {
                LocalVar->PitComAct[K] = seclpfilter_c(
                    LocalVar->PitCom[K], LocalVar->DT,
                    CntrPar->PA_CornerFreq, CntrPar->PA_Damping,
                    &LocalVar->FP, LocalVar->iStatus,
                    (LocalVar->restart != 0), &objInst->instSecLPF,
                    0, 0.0);
            }
        } else {
            LocalVar->PitComAct[K] = LocalVar->PitCom[K];
        }
    }

    // Hardware saturation
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->PitComAct[K] = saturate_c(LocalVar->PitComAct[K], CntrPar->PC_MinPit, CntrPar->PC_MaxPit);
        LocalVar->PitComAct[K] = ratelimit_c(LocalVar->PitComAct[K], CntrPar->PC_MinRat, CntrPar->PC_MaxRat,
                                              LocalVar->DT, (LocalVar->restart != 0),
                                              &LocalVar->rlP, &objInst->instRL,
                                              1, LocalVar->BlPitch[K]);
    }

    // Pitch fault modes
    if (CntrPar->PF_Mode == 1) {
        for (int K = 0; K < LocalVar->NumBl; K++) {
            LocalVar->PitComAct[K] += CntrPar->PF_Offsets[K];
        }
    } else if (CntrPar->PF_Mode == 2) {
        for (int K = 0; K < LocalVar->NumBl; K++) {
            if (LocalVar->Time > CntrPar->PF_TimeStuck[K]) {
                LocalVar->PitComAct[K] = LocalVar->BlPitch[K];
            }
        }
    }

    // Command pitch to avrSWAP (Fortran 42-45 → C 41-44)
    avrSWAP[41] = LocalVar->PitComAct[0];
    avrSWAP[42] = LocalVar->PitComAct[1];
    avrSWAP[43] = LocalVar->PitComAct[2];
    avrSWAP[44] = LocalVar->PitComAct[0];  // Collective pitch = blade 1

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "PitchControl:%s", ErrVar->ErrMsg);
        strncpy(ErrVar->ErrMsg, tmp, sizeof(ErrVar->ErrMsg) - 1);
        ErrVar->ErrMsg[sizeof(ErrVar->ErrMsg) - 1] = '\0';
    }
}
