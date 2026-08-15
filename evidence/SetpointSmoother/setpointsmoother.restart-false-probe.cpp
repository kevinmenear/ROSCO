// P10 PROBE -- NOT the translation. `restart ? 1 : 0` -> `restart ? 0 : 0`.
// The reset argument is forced to 0, so this fails exactly the cases in which
// `restart` is TRUE and that truth reaches LPFilter's `(iStatus == 0) ||
// (reset != 0)`. Its count is the set that mutant `restart ? 2 : 0` is
// declared equivalent OVER; if it were 0, that mutant would survive because
// the corpus never builds a 2, which is a claim about the corpus and not an
// equivalence.
// VIT Translation Scaffold
// Function: SetpointSmoother
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 00345d404b2f
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T03:27:49Z
//
// Unit #40. `ControllerBlocks.f90:490-517` in the clean source.
//
// The whole unit is two arms and four statements, and both arms write exactly
// one compared field, `LocalVar%SS_DelOmegaF`.
//
// THE `/` AND `*` IN THE SECOND TERM ASSOCIATE LEFT TO RIGHT AND THE ALGEBRA
// IS NOT THE SHAPE. The reference writes
//
//     ... - ((VS_RtPwr * R_Total - VS_LastGenPwr))/VS_RtPwr * SS_PCGain
//
// which is `((num / VS_RtPwr) * SS_PCGain)`, a divide and then a multiply --
// NOT `num * SS_PCGain / VS_RtPwr` and not `num / (VS_RtPwr / SS_PCGain)`.
// Transcribed here in that order, parenthesised so nothing later re-associates
// it. `0.524` divides before its multiply for the same reason.
//
// `LocalVar%FP` is a nested derived type held BY VALUE inside the view struct,
// so its address is taken here and the wrapper must copy it back: this unit
// integrates with `--reverse-copy`. It is the callee, not this unit, that
// writes it -- the same shape unit #38 recorded.

#include "vit_types.h"

void SetpointSmoother(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst) {
    // REAL(DbKi) :: DelOmega   ! Reference generator speed shift, rad/s.
    // REAL(DbKi) :: R_Total    ! Total power rating command
    double DelOmega;
    double R_Total;

    // ! ------ Setpoint Smoothing ------
    // IF ( CntrPar%SS_Mode == 1) THEN
    if (CntrPar->SS_Mode == 1) {
        // ! Find setpoint shift amount
        // R_Total = LocalVar%PRC_R_Speed * LocalVar%PRC_R_Torque * LocalVar%PRC_R_Pitch
        R_Total = LocalVar->PRC_R_Speed * LocalVar->PRC_R_Torque * LocalVar->PRC_R_Pitch;

        // DelOmega = ((LocalVar%BlPitchCMeas - LocalVar%PC_MinPit)/0.524) * CntrPar%SS_VSGain
        //          - ((CntrPar%VS_RtPwr * R_Total - LocalVar%VS_LastGenPwr))/CntrPar%VS_RtPwr
        //            * CntrPar%SS_PCGain   ! Normalize to 30 degrees for now
        DelOmega = ((LocalVar->BlPitchCMeas - LocalVar->PC_MinPit) / 0.524) * CntrPar->SS_VSGain
                   - ((CntrPar->VS_RtPwr * R_Total - LocalVar->VS_LastGenPwr) / CntrPar->VS_RtPwr)
                         * CntrPar->SS_PCGain;

        // DelOmega = DelOmega * CntrPar%PC_RefSpd
        DelOmega = DelOmega * CntrPar->PC_RefSpd;

        // ! Filter
        // LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, CntrPar%F_SSCornerFreq,
        //                                  LocalVar%FP, LocalVar%iStatus, LocalVar%restart,
        //                                  objInst%instLPF)
        //
        // `InitialValue` is OPTIONAL in the reference and is NOT passed here,
        // so the bridge's presence flag is 0 and its value is unread.
        LocalVar->SS_DelOmegaF =
            lpfilter_c(DelOmega, LocalVar->DT, CntrPar->F_SSCornerFreq, &LocalVar->FP,
                       LocalVar->iStatus, LocalVar->restart ? 0 : 0, &objInst->instLPF, 0, 0.0);
    } else {
        // LocalVar%SS_DelOmegaF = 0 ! No setpoint smoothing
        LocalVar->SS_DelOmegaF = 0;
    }
}
