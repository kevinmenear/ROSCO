// VIT Translation
// Function: CableControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE CableControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
// Source MD5: 02994a62d8d2
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-24T16:04:46Z

#include "controlparameters_view_t.h"
#include "errorvariables_t.h"
#include "filterparameters_t.h"
#include "objectinstances_t.h"
#include "piparams_t.h"
#include "resparams_t.h"
#include "rlparams_t.h"
#include "we_t.h"
#include "localvariables_t.h"

#include <cstring>
#include <cstdio>
#include "rosco_constants.h"

// Callee entry points
extern "C" {
    double interp1d_c(double* xData, int n_xData, double* yData, int n_yData,
                      double xq, errorvariables_t* ErrVar);
    double seclpfilter_vel_c(double InputSignal, double DT, double CornerFreq,
                             double Damp, filterparameters_t* FP, int iStatus,
                             int reset, int* inst, int has_InitialValue,
                             double InitialValue);
    double picontroller_c(double error, double kp, double ki,
                          double minValue, double maxValue, double DT,
                          double I0, piparams_t* piP, int reset, int* inst);
}

void CableControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, errorvariables_t* ErrVar) {
    // CableControl: cable length control
    //   CC_Mode = 1: user-defined step inputs
    //   CC_Mode = 2: open-loop from lookup table

    if (CntrPar->CC_Mode == 1) {
        // User-defined control — step change at t > 500
        if (LocalVar->Time > 500) {
            LocalVar->CC_DesiredL[0] = -14.51;
            LocalVar->CC_DesiredL[1] = 1.58;
            LocalVar->CC_DesiredL[2] = -10.332;
        }

    } else if (CntrPar->CC_Mode == 2) {
        // Open-loop control
        for (int I_GROUP = 0; I_GROUP < CntrPar->CC_Group_N; I_GROUP++) {
            if (CntrPar->Ind_CableControl[I_GROUP] > 0) {
                // Extract row from column-major 2D array
                int n_rows = CntrPar->n_OL_CableControl_rows;
                int n_cols = CntrPar->n_OL_CableControl_cols;
                double row_slice[n_cols];
                for (int col = 0; col < n_cols; col++) {
                    row_slice[col] = CntrPar->OL_CableControl[col * n_rows + I_GROUP];
                }
                LocalVar->CC_DesiredL[I_GROUP] = interp1d_c(
                    CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                    row_slice, n_cols,
                    LocalVar->Time, ErrVar);
            }
        }
    }

    // Convert desired to actuated line length and delta length for all groups
    for (int I_GROUP = 0; I_GROUP < CntrPar->CC_Group_N; I_GROUP++) {
        // Get actuated deltaL via second-order low-pass filter
        LocalVar->CC_ActuatedDL[I_GROUP] = seclpfilter_vel_c(
            LocalVar->CC_DesiredL[I_GROUP],
            LocalVar->DT,
            2.0 * PI / CntrPar->CC_ActTau,   // CornerFreq
            1.0,                                  // Damp
            &LocalVar->FP,
            LocalVar->iStatus,
            (LocalVar->restart != 0),
            &objInst->instSecLPFV,
            0, 0.0  // no InitialValue
        );

        // Integrate delta-L to get actuated length
        LocalVar->CC_ActuatedL[I_GROUP] = picontroller_c(
            LocalVar->CC_ActuatedDL[I_GROUP],
            0.0,                                  // kp
            1.0,                                  // ki (pure integrator)
            -1000.0,                              // minValue
            1000.0,                               // maxValue
            LocalVar->DT,
            LocalVar->CC_ActuatedDL[0],           // I0: initial from first group
            &LocalVar->piP,
            (LocalVar->restart != 0),
            &objInst->instPI
        );
    }

    // Assign to avrSWAP
    for (int I_GROUP = 0; I_GROUP < CntrPar->CC_Group_N; I_GROUP++) {
        // Fortran: avrSWAP(CC_GroupIndex(I_GROUP)) and +1, 1-indexed
        int idx = CntrPar->CC_GroupIndex[I_GROUP] - 1;  // 0-indexed
        avrSWAP[idx] = LocalVar->CC_ActuatedL[I_GROUP];
        avrSWAP[idx + 1] = LocalVar->CC_ActuatedDL[I_GROUP];
    }

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "CableControl:%s", ErrVar->ErrMsg);
        strncpy(ErrVar->ErrMsg, tmp, sizeof(ErrVar->ErrMsg) - 1);
        ErrVar->ErrMsg[sizeof(ErrVar->ErrMsg) - 1] = '\0';
    }
}
