// VIT Translation
// Function: SetParameters (partial — Fortran wrapper handles per-timestep init,
//           banner, accINFILE save, ReadControlParameterFileSub, ReadCpFile)
// Source: ReadSetParameters.f90
// Module: ReadSetParameters
// Status: unverified

#include "vit_types.h"
#include "rosco_constants.h"
#include <cstring>
#include <algorithm>

// Callee entry points
extern "C" {
    void checkinputs_c(localvariables_t* LocalVar, controlparameters_view_t* CntrPar,
                       float* avrSWAP, errorvariables_t* ErrVar, int32_t size_avcMSG);
    double interp1d_c(double* xData, int n_xData, double* yData, int n_yData,
                      double xq, errorvariables_t* ErrVar);
    double lpfilter_c(double InputSignal, double DT, double CornerFreq,
                      filterparameters_t* FP, int iStatus, int reset, int* inst,
                      int has_InitialValue, double InitialValue);
}

void SetParameters(controlparameters_view_t* CntrPar, localvariables_t* LocalVar,
                   float* avrSWAP, objectinstances_t* objInst,
                   errorvariables_t* ErrVar, int size_avcMSG) {

    // iStatus==0: Initialize LocalVar fields (CntrPar is populated by wrapper)
    if (LocalVar->iStatus == 0) {
        // PitCom = BlPitch
        for (int K = 0; K < 3; K++) {
            LocalVar->PitCom[K] = LocalVar->BlPitch[K];
        }

        // Wind speed estimator initialization
        LocalVar->WE_Vw = LocalVar->HorWindV;
        LocalVar->WE_VwI = LocalVar->WE_Vw - CntrPar->WE_Gamma * LocalVar->RotSpeed;
        LocalVar->WE_Op = 1;
        LocalVar->WE_Op_Last = 1;

        // Setpoint Smoother initialization
        LocalVar->SS_DelOmegaF = 0;

        // Generator torque initial condition
        if (CntrPar->VS_FBP == VS_FBP_Variable_Pitch) {
            if (LocalVar->GenSpeed > 0.98 * CntrPar->PC_RefSpd) {
                LocalVar->GenTq = CntrPar->VS_RtTq;
            } else {
                LocalVar->GenTq = std::min(CntrPar->VS_RtTq,
                    CntrPar->VS_Rgn2K * LocalVar->GenSpeed * LocalVar->GenSpeed);
            }
        } else {
            LocalVar->GenTq = interp1d_c(CntrPar->VS_FBP_U, CntrPar->n_VS_FBP_U,
                                         CntrPar->VS_FBP_Tau, CntrPar->n_VS_FBP_Tau,
                                         LocalVar->HorWindV, ErrVar);
        }
        LocalVar->VS_LastGenTrq = LocalVar->GenTq;
        LocalVar->VS_MaxTq = CntrPar->VS_MaxTq;
        LocalVar->VS_GenPwr = LocalVar->GenTq * LocalVar->GenSpeed * CntrPar->VS_GenEff / 100.0;

        // Initialize cable/structural control variables
        memset(LocalVar->CC_DesiredL, 0, sizeof(LocalVar->CC_DesiredL));
        memset(LocalVar->CC_ActuatedL, 0, sizeof(LocalVar->CC_ActuatedL));
        memset(LocalVar->CC_ActuatedDL, 0, sizeof(LocalVar->CC_ActuatedDL));
        memset(LocalVar->StC_Input, 0, sizeof(LocalVar->StC_Input));

        LocalVar->ZMQ_YawOffset = 0;
        memset(LocalVar->ZMQ_PitOffset, 0, sizeof(LocalVar->ZMQ_PitOffset));
        LocalVar->ZMQ_ID = CntrPar->ZMQ_ID;

        // Check validity of input parameters
        checkinputs_c(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG);
    }

    // Per-timestep: Open Loop index
    if (CntrPar->OL_BP_Mode == 0) {
        LocalVar->OL_Index = LocalVar->Time;
    } else {
        LocalVar->OL_Index = LocalVar->WE_Vw;
        if (CntrPar->OL_BP_FiltFreq > 0) {
            LocalVar->OL_Index = lpfilter_c(LocalVar->WE_Vw, LocalVar->DT,
                CntrPar->OL_BP_FiltFreq, &LocalVar->FP, LocalVar->iStatus,
                (LocalVar->restart != 0), &objInst->instLPF, 0, 0.0);
        }
    }
}

// extern "C" wrapper — manual integration
extern "C" {
    void setparameters_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar,
                         float* avrSWAP, objectinstances_t* objInst,
                         errorvariables_t* ErrVar, int size_avcMSG) {
        SetParameters(CntrPar, LocalVar, avrSWAP, objInst, ErrVar, size_avcMSG);
    }
}
