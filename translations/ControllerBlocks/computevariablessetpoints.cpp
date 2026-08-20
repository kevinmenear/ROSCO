// VIT Translation Scaffold
// Function: ComputeVariablesSetpoints
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 0b9bc4f3ff4d
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-20T13:03:10Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// FOUR CALLEES, ALL FOUR ALREADY TRANSLATED AND ALL FOUR CALLED RATHER THAN
// INLINED (X1): `LPFilter` (#12, x2), `interp1d` (#23, x0..3),
// `RefSpeedExclusion` (#59, x0..1), `saturate` (#24, x0..1). Two of the calls
// -- the two `LPFilter`s -- post-increment `objInst%instLPF`, so the ORDER of
// the calls decides which `LocalVar%FP` slot each filter owns. The second
// LPFilter runs on EVERY path; the first only under `PRC_Mode == 1`, so which
// slot the second filter reads is a function of `PRC_Mode`.
//
// ARM COVERAGE, read from coverage/line_coverage.json over all 27 scenarios,
// against the CLEAN source (54dd134) line numbers:
//
//   :102 IF (PRC_Mode == 1)               407,976 in 23 scenarios
//   :103 the LPFilter                      39,998 in  2   <- scenarios 26, 27
//   :104 the interp1d                      39,998 in  2
//   :109 SS_DelOmegaF < 0                 218,425 in 22
//   :111 the ELSE                         189,551 in 23
//   :123 VS_Mode_WSE_TSR                  391,977 in 22
//   :126 VS_Mode_Power_TSR                     0            <- NO SCENARIO
//   :129 VS_Mode_Torque_TSR                    0            <- NO SCENARIO
//   :132 the ELSE (constant reference)     15,999 in  1
//   :137 IF (VS_RefSpd_TSR > VS_RefSpd)   407,976 in 23
//   :139 the VS_FBP_WSE_Ref lookup              0            <- NO SCENARIO
//   :142 the VS_FBP_Torque_Ref lookup           0            <- NO SCENARIO
//   :157 CALL RefSpeedExclusion            11,999 in  1
//   :162 the saturate                     407,976 in 23
//   :167 the PRC_Mode interp1d             39,998 in  2
//   :172 SS_DelOmegaF > 0                 173,528 in 16
//
// So FOUR of this unit's statements are outside everything the 27-scenario
// simulation gate drives -- the two alternative torque-control laws and both
// Region-3 feed-back-pitch lookups. Only the differential harness reaches them,
// and that asymmetry is the reason P11 is not optional here.
//
// ONE STATEMENT IS A DEAD STORE IN THE REFERENCE, AND IT IS TRANSCRIBED ANYWAY.
// `:148  LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_TSR * LocalVar%PRC_R_Speed`
// is overwritten four lines later by the LPFilter assignment, which reads
// `VS_RefSpd_TSR` and not `VS_RefSpd`, with nothing in between that reads
// `VS_RefSpd`. The mirror contract is about the reference's behaviour, not
// about the part of it some instrument can distinguish, so the multiply stays
// (P7) -- and the mutants it offers are declared, with the control for the
// declaration being the SAME multiply at `:162`, inside `saturate`'s third
// argument, whose mutants are NOT declared because that one is read.

#include "vit_types.h"

#include <algorithm>
#include <cmath>

namespace {

// Constants.f90:42-46 and :53-56. Copied as the reference declares them; the
// Fortran compares `VS_ControlMode` and `VS_FBP` against these NAMES, so the
// translation compares against the same values rather than against bare
// literals -- a `const_tweak` on one of these is then a mutant on the same
// quantity the reference names.
constexpr int VS_Mode_WSE_TSR = 2;
constexpr int VS_Mode_Power_TSR = 3;
constexpr int VS_Mode_Torque_TSR = 4;

constexpr int VS_FBP_Variable_Pitch = 0;
constexpr int VS_FBP_WSE_Ref = 2;
constexpr int VS_FBP_Torque_Ref = 3;

}  // namespace

void ComputeVariablesSetpoints(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                               objectinstances_t* objInst, debugvariables_t* DebugVar,
                               errorvariables_view_t* ErrVar) {
    // !   Change pitch reference speed
    // LocalVar%PC_RefSpd_PRC = CntrPar%PC_RefSpd * LocalVar%PRC_R_Speed
    LocalVar->PC_RefSpd_PRC = CntrPar->PC_RefSpd * LocalVar->PRC_R_Speed;

    // ! Lookup table for speed setpoint (PRC_Mode 1)
    // IF (CntrPar%PRC_Mode == 1) THEN
    if (CntrPar->PRC_Mode == 1) {
        // LocalVar%PRC_WSE_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT,
        //     CntrPar%PRC_LPF_Freq, LocalVar%FP, LocalVar%iStatus,
        //     LocalVar%restart, objInst%instLPF)
        //
        // Seven actual arguments: LPFilter's eighth dummy, `InitialValue`, is
        // OPTIONAL and not supplied, so the bridge's `has_InitialValue` is 0
        // and the value it carries is never read. The `? 1 : 0` on `restart`
        // is the campaign's standing spelling for a Fortran LOGICAL crossing
        // into the `int32_t` the bridge declares (startup.cpp, shutdown.cpp).
        LocalVar->PRC_WSE_F =
            lpfilter_c(LocalVar->WE_Vw, LocalVar->DT, CntrPar->PRC_LPF_Freq, &LocalVar->FP,
                       LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
        // LocalVar%PC_RefSpd_PRC = interp1d(CntrPar%PRC_WindSpeeds,
        //     CntrPar%PRC_GenSpeeds, LocalVar%PRC_WSE_F, ErrVar)
        LocalVar->PC_RefSpd_PRC =
            interp1d_c(CntrPar->PRC_WindSpeeds, CntrPar->n_PRC_WindSpeeds,
                       CntrPar->PRC_GenSpeeds, CntrPar->n_PRC_GenSpeeds,
                       LocalVar->PRC_WSE_F, ErrVar);
    // ENDIF
    }

    // ! Implement setpoint smoothing
    // IF (LocalVar%SS_DelOmegaF < 0) THEN
    //     LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC - LocalVar%SS_DelOmegaF
    // ELSE
    //     LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC
    // ENDIF
    //
    // The two arms are NOT the same computation at the boundary and the
    // difference is a signed zero: at `SS_DelOmegaF == -0.0` the test is false,
    // so the ELSE runs and the answer is `PC_RefSpd_PRC`, while the THEN arm
    // would have computed `PC_RefSpd_PRC - (-0.0)`, which differs from it only
    // when `PC_RefSpd_PRC` is itself `-0.0`. Written as the reference writes
    // it, both arms present.
    if (LocalVar->SS_DelOmegaF < 0) {
        LocalVar->PC_RefSpd_SS = LocalVar->PC_RefSpd_PRC - LocalVar->SS_DelOmegaF;
    } else {
        LocalVar->PC_RefSpd_SS = LocalVar->PC_RefSpd_PRC;
    }

    // ! Compute error for pitch controller
    // LocalVar%PC_RefSpd = LocalVar%PC_RefSpd_SS
    LocalVar->PC_RefSpd = LocalVar->PC_RefSpd_SS;
    // LocalVar%PC_SpdErr = LocalVar%PC_RefSpd - LocalVar%GenSpeedF   ! Speed error
    LocalVar->PC_SpdErr = LocalVar->PC_RefSpd - LocalVar->GenSpeedF;
    // LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr     ! Power error, unused
    LocalVar->PC_PwrErr = CntrPar->VS_RtPwr - LocalVar->VS_GenPwr;

    // ! ----- Torque controller reference errors -----
    // ! Define VS reference generator speed [rad/s]
    // IF (CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) THEN
    if (CntrPar->VS_ControlMode == VS_Mode_WSE_TSR) {
        // ! Use unfiltered wind speed estimate, then filter below
        // LocalVar%VS_RefSpd_TSR = (CntrPar%VS_TSRopt * LocalVar%WE_Vw
        //                           / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio
        //
        // THE PARENTHESES ARE THE REFERENCE'S OWN and they are load-bearing.
        // `*` and `/` associate left to right at equal precedence, so the
        // bracketed part is `(VS_TSRopt * WE_Vw) / WE_BladeRadius` and the
        // gearbox ratio multiplies the QUOTIENT. Reassociating -- multiplying
        // the two configuration constants first, say -- is a different rounding
        // (unit #48's `assoc_reorder` margin was one case in 1,131).
        LocalVar->VS_RefSpd_TSR =
            (CntrPar->VS_TSRopt * LocalVar->WE_Vw / CntrPar->WE_BladeRadius) *
            CntrPar->WE_GearboxRatio;

    // ELSEIF (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) THEN
    } else if (CntrPar->VS_ControlMode == VS_Mode_Power_TSR) {
        // LocalVar%VS_RefSpd_TSR = (MAX(LocalVar%VS_GenPwr, 0.0)
        //                           /(CntrPar%VS_GenEff/100.0)/CntrPar%VS_Rgn2K)**(1./3.)
        //
        // `1./3.` is a default-REAL expression and the file is compiled with
        // `-fdefault-real-8`, so the exponent is the REAL(8) nearest one third
        // -- NOT an integer 1/3, which would be 0. A REAL exponent makes
        // gfortran emit a libm `pow` call (the check registry's rule about
        // `x**3.0`), so this is `std::pow` and not a hand-rolled cube root.
        //
        // `MAX` is `std::max` and not `std::fmax`: gfortran lowers the MAX
        // intrinsic to a MAX_EXPR, which is `(a < b) ? b : a` in the operand
        // order written, and that is exactly `std::max`. `std::fmax` is a libm
        // call that PREFERS THE NON-NaN OPERAND, which differs from the
        // reference at a NaN input -- and a NaN input is on R6's own ladder, so
        // this corpus reaches the difference.
        //
        // Both divisions are transcribed as divisions in the reference's order:
        // `a / b / c` is `(a / b) / c`, which rounds differently from
        // `a / (b * c)`.
        LocalVar->VS_RefSpd_TSR =
            std::pow(std::max(LocalVar->VS_GenPwr, 0.0) / (CntrPar->VS_GenEff / 100.0) /
                         CntrPar->VS_Rgn2K,
                     1. / 3.);

    // ELSEIF (CntrPar%VS_ControlMode == VS_Mode_Torque_TSR) THEN
    } else if (CntrPar->VS_ControlMode == VS_Mode_Torque_TSR) {
        // LocalVar%VS_RefSpd_TSR = (MAX(LocalVar%GenTq, 0.0)/CntrPar%VS_Rgn2K)**(1./2.)
        //
        // `1./2.` is 0.5 EXACTLY, and it is still a REAL exponent -- so this is
        // `std::pow(x, 0.5)` and NOT `std::sqrt(x)`. The two differ: `pow`
        // rounds through its own path and `sqrt` is correctly rounded, and GCC
        // only rewrites one into the other under `-funsafe-math-optimizations`,
        // which this build does not set. Transcribing it as a square root would
        // be the algebra rather than the shape.
        LocalVar->VS_RefSpd_TSR =
            std::pow(std::max(LocalVar->GenTq, 0.0) / CntrPar->VS_Rgn2K, 1. / 2.);

    // ELSE ! Generate constant speed reference if K*Omega^2 in use or torque
    //      ! control disabled
    } else {
        // LocalVar%VS_RefSpd_TSR = CntrPar%VS_RefSpd
        LocalVar->VS_RefSpd_TSR = CntrPar->VS_RefSpd;
    // ENDIF
    }

    // ! Region 3 FBP reference logic, triggers if Region-2 reference speed is
    // ! higher than rated
    // IF (LocalVar%VS_RefSpd_TSR > CntrPar%VS_RefSpd) THEN
    if (LocalVar->VS_RefSpd_TSR > CntrPar->VS_RefSpd) {
        // IF (CntrPar%VS_FBP == VS_FBP_WSE_Ref) THEN
        if (CntrPar->VS_FBP == VS_FBP_WSE_Ref) {
            // LocalVar%VS_RefSpd_TSR = interp1d(CntrPar%VS_FBP_U,
            //     CntrPar%VS_FBP_Omega, LocalVar%WE_Vw, ErrVar)
            LocalVar->VS_RefSpd_TSR =
                interp1d_c(CntrPar->VS_FBP_U, CntrPar->n_VS_FBP_U,
                           CntrPar->VS_FBP_Omega, CntrPar->n_VS_FBP_Omega,
                           LocalVar->WE_Vw, ErrVar);

        // ELSEIF (CntrPar%VS_FBP == VS_FBP_Torque_Ref) THEN
        } else if (CntrPar->VS_FBP == VS_FBP_Torque_Ref) {
            // LocalVar%VS_RefSpd_TSR = interp1d(CntrPar%VS_FBP_Tau,
            //     CntrPar%VS_FBP_Omega, LocalVar%GenTq, ErrVar)
            LocalVar->VS_RefSpd_TSR =
                interp1d_c(CntrPar->VS_FBP_Tau, CntrPar->n_VS_FBP_Tau,
                           CntrPar->VS_FBP_Omega, CntrPar->n_VS_FBP_Omega,
                           LocalVar->GenTq, ErrVar);

        // ENDIF
        //
        // NO THIRD ARM. `VS_FBP` values 0 and 1 fall through both tests and
        // `VS_RefSpd_TSR` keeps the value the control-law chain gave it, which
        // is a real and reachable behaviour of the reference (P6/P7).
        }
    // ENDIF
    }

    // ! Change VS Ref speed based on R_Speed
    // LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_TSR * LocalVar%PRC_R_Speed
    //
    // THE DEAD STORE. See the header: the next statement that touches
    // `VS_RefSpd` is the LPFilter assignment below, which overwrites it from
    // `VS_RefSpd_TSR`. Transcribed rather than dropped.
    LocalVar->VS_RefSpd = LocalVar->VS_RefSpd_TSR * LocalVar->PRC_R_Speed;

    // ! Filter reference signal
    // LocalVar%VS_RefSpd = LPFilter(LocalVar%VS_RefSpd_TSR, LocalVar%DT,
    //     CntrPar%F_VSRefSpdCornerFreq, LocalVar%FP, LocalVar%iStatus,
    //     LocalVar%restart, objInst%instLPF)
    //
    // The INPUT is `VS_RefSpd_TSR`, not the product just stored. That is what
    // makes the statement above dead, and it is transcribed from the reference
    // rather than "corrected" to read the assignment above it.
    LocalVar->VS_RefSpd =
        lpfilter_c(LocalVar->VS_RefSpd_TSR, LocalVar->DT, CntrPar->F_VSRefSpdCornerFreq,
                   &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0,
                   &objInst->instLPF, 0, 0.0);

    // ! Exclude reference speeds specified by user
    // IF (CntrPar%TRA_Mode > 0) THEN
    //     CALL RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
    // END IF
    //
    // CALLED, not inlined (X1). The argument ORDER is the callee's own --
    // LocalVar first, CntrPar second -- which is the reverse of this unit's own
    // signature.
    if (CntrPar->TRA_Mode > 0) {
        refspeedexclusion_c(LocalVar, CntrPar, objInst, DebugVar);
    }

    // ! Saturate torque reference speed below rated speed if using pitch
    // ! control in Region 3
    // IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
    //     LocalVar%VS_RefSpd = saturate(LocalVar%VS_RefSpd, CntrPar%VS_MinOMSpd,
    //                                   CntrPar%VS_RefSpd * LocalVar%PRC_R_Speed)
    // END IF
    //
    // The same `<something> * PRC_R_Speed` product as the dead store above, and
    // this one IS read -- it is `saturate`'s upper bound. The pair is the
    // control for that declaration.
    if (CntrPar->VS_FBP == VS_FBP_Variable_Pitch) {
        LocalVar->VS_RefSpd = saturate_c(LocalVar->VS_RefSpd, CntrPar->VS_MinOMSpd,
                                         CntrPar->VS_RefSpd * LocalVar->PRC_R_Speed);
    }

    // ! Simple lookup table for generator speed (PRC_Mode 1)
    // IF (CntrPar%PRC_Mode == 1) THEN
    //     LocalVar%VS_RefSpd = interp1d(CntrPar%PRC_WindSpeeds,
    //         CntrPar%PRC_GenSpeeds, LocalVar%PRC_WSE_F, ErrVar)
    // ENDIF
    //
    // Reads the `PRC_WSE_F` the FIRST block wrote -- and reads it under the
    // same predicate, so the two are always in step. It is a SECOND lookup on
    // the same pair of tables at the same abscissa as the one that produced
    // `PC_RefSpd_PRC`, and the reference does it twice rather than reusing the
    // first answer; so does this.
    if (CntrPar->PRC_Mode == 1) {
        LocalVar->VS_RefSpd =
            interp1d_c(CntrPar->PRC_WindSpeeds, CntrPar->n_PRC_WindSpeeds,
                       CntrPar->PRC_GenSpeeds, CntrPar->n_PRC_GenSpeeds,
                       LocalVar->PRC_WSE_F, ErrVar);
    }

    // ! Implement setpoint smoothing
    // IF (LocalVar%SS_DelOmegaF > 0) THEN
    //     LocalVar%VS_RefSpd = LocalVar%VS_RefSpd - LocalVar%SS_DelOmegaF
    // ENDIF
    //
    // NOT the mirror of the pitch-side smoothing above: that one tests `< 0`
    // and has an ELSE, this one tests `> 0` and has none. So `SS_DelOmegaF == 0`
    // takes the ELSE there and no branch here, and the two sides subtract on
    // OPPOSITE signs.
    if (LocalVar->SS_DelOmegaF > 0) {
        LocalVar->VS_RefSpd = LocalVar->VS_RefSpd - LocalVar->SS_DelOmegaF;
    }

    // ! Force minimum rotor speed
    // LocalVar%VS_RefSpd = max(LocalVar%VS_RefSpd, CntrPar%VS_MinOmSpd)
    //
    // `VS_MinOmSpd` and `VS_MinOMSpd` are the same field: Fortran identifiers
    // are case-insensitive and the reference spells it both ways within four
    // lines. The C view struct has one spelling, `VS_MinOMSpd`.
    LocalVar->VS_RefSpd = std::max(LocalVar->VS_RefSpd, CntrPar->VS_MinOMSpd);

    // ! Compute speed error from reference
    // LocalVar%VS_SpdErr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF
    LocalVar->VS_SpdErr = LocalVar->VS_RefSpd - LocalVar->GenSpeedF;

    // ! Define transition region setpoint errors
    // LocalVar%VS_SpdErrAr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF
    //     ! Current speed error - Region 2.5 PI-control (Above Rated)
    //
    // The same expression as `VS_SpdErr` one line up, stored into a second
    // field. Both are outputs, so both are compared, and the two are equal on
    // every input by construction.
    LocalVar->VS_SpdErrAr = LocalVar->VS_RefSpd - LocalVar->GenSpeedF;
    // LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF
    //     ! Current speed error - Region 1.5 PI-control (Below Rated)
    LocalVar->VS_SpdErrBr = CntrPar->VS_MinOMSpd - LocalVar->GenSpeedF;

    // ! Region 3 minimum pitch angle for state machine
    // LocalVar%VS_Rgn3Pitch = LocalVar%PC_MinPit + CntrPar%PC_Switch
    LocalVar->VS_Rgn3Pitch = LocalVar->PC_MinPit + CntrPar->PC_Switch;

    // ! Debug Vars
    // DebugVar%VS_RefSpd = LocalVar%VS_RefSpd
    DebugVar->VS_RefSpd = LocalVar->VS_RefSpd;
    // DebugVar%PC_RefSpd = LocalVar%PC_RefSpd
    DebugVar->PC_RefSpd = LocalVar->PC_RefSpd;
}
