// VIT Translation
// Function: YawRateControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
// Source MD5: 9ebb1ecfc720
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-24T17:13:45Z

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

// Callee entry points
extern "C" {
    double wrap_180_c(double x);
    double lpfilter_c(double InputSignal, double DT, double CornerFreq,
                      filterparameters_t* FP, int iStatus, int reset, int* inst,
                      int has_InitialValue, double InitialValue);
    double interp1d_c(double* xData, int n_xData, double* yData, int n_yData,
                      double xq, errorvariables_t* ErrVar);
}

void YawRateControl(float* avrSWAP, controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar) {
    // YawRateControl: yaw rate control using yaw drive
    //   Y_ControlMode = 1: active yaw rate control

    // SAVE variables — persist across calls
    static double NacVaneOffset = 0.0;
    static int YawState = 0;
    static double NacHeadingError = 0.0;
    static int Tidx = 0;

    if (CntrPar->Y_ControlMode == 1) {
        // Compass wind direction in degrees
        LocalVar->WindDir = wrap_180_c(LocalVar->NacHeading + LocalVar->NacVane);

        // Initialize
        if (LocalVar->iStatus == 0) {
            YawState = 0;
            Tidx = 1;  // Fortran 1-indexed, used as-is in debug output
        }

        // Compute/apply offset
        if (CntrPar->ZMQ_Mode == 1) {
            NacVaneOffset = LocalVar->ZMQ_YawOffset;
        } else {
            NacVaneOffset = CntrPar->Y_MErrSet;
        }

        // Update filtered wind direction
        double WindDirPlusOffset = wrap_180_c(LocalVar->WindDir + NacVaneOffset);
        double WindDirPlusOffsetCosF = lpfilter_c(
            cos(WindDirPlusOffset * D2R), LocalVar->DT, CntrPar->F_YawErr,
            &LocalVar->FP, LocalVar->iStatus, 0, &objInst->instLPF,
            0, 0.0);
        double WindDirPlusOffsetSinF = lpfilter_c(
            sin(WindDirPlusOffset * D2R), LocalVar->DT, CntrPar->F_YawErr,
            &LocalVar->FP, LocalVar->iStatus, 0, &objInst->instLPF,
            0, 0.0);
        double NacHeadingTarget = wrap_180_c(atan2(WindDirPlusOffsetSinF, WindDirPlusOffsetCosF) * R2D);

        // Yaw error
        NacHeadingError = wrap_180_c(NacHeadingTarget - LocalVar->NacHeading);

        // Check for deadband
        double deadband;
        if (LocalVar->WE_Vw_F <= CntrPar->Y_uSwitch) {
            deadband = CntrPar->Y_ErrThresh[0];  // Fortran(1) → C[0]
        } else {
            deadband = CntrPar->Y_ErrThresh[1];  // Fortran(2) → C[1]
        }

        // Yaw state machine
        double YawRateCom;
        if (YawState == 1) {
            // Yawing right
            if (NacHeadingError <= 0.0) {
                YawRateCom = 0.0;
                YawState = 0;
            } else {
                YawRateCom = CntrPar->Y_Rate;
                YawState = 1;
            }
        } else if (YawState == -1) {
            // Yawing left
            if (NacHeadingError >= 0.0) {
                YawRateCom = 0.0;
                YawState = 0;
            } else {
                YawRateCom = -CntrPar->Y_Rate;
                YawState = -1;
            }
        } else {
            // Stopped — initiate yaw if outside deadband
            if (NacHeadingError > deadband) {
                YawState = 1;
            }
            if (NacHeadingError < -deadband) {
                YawState = -1;
            }
            YawRateCom = 0.0;
        }

        // Output yaw rate command in rad/s (Fortran avrSWAP(48) → C [47])
        avrSWAP[47] = YawRateCom * D2R;

        // Open loop yaw rate control override
        if ((CntrPar->OL_Mode > 0) && (CntrPar->Ind_YawRate > 0)) {
            if (LocalVar->Time >= CntrPar->OL_Breakpoints[0]) {
                avrSWAP[47] = interp1d_c(
                    CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                    CntrPar->OL_YawRate, CntrPar->n_OL_YawRate,
                    LocalVar->OL_Index, ErrVar);
            }
        }

        // Save for debug
        DebugVar->YawRateCom = YawRateCom;
        DebugVar->NacHeadingTarget = NacHeadingTarget;
        DebugVar->NacVaneOffset = NacVaneOffset;
        DebugVar->YawState = YawState;
        DebugVar->Yaw_Err = NacHeadingError;
    }
}
