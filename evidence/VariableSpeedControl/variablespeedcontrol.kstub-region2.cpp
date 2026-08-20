// KERNEL RED TEST STUB for unit #60 VariableSpeedControl -- the K*Omega^2 Region-2 arm doubled
//
// NOT A TRANSLATION. `vit verify` printed NON_DISCRIMINATING for this unit --
// "no by-value floating-point parameter and no floating-point result: every
// input arrives behind a pointer or inside a derived type" -- the same refusal
// it gave units #40 through #44. So the kernel's 60/60 has to be red-tested by
// hand or its discriminating power stays unmeasured.
//
// `LocalVar%GenTq = LocalVar%VS_KOmega2_GenTq` becomes `... * 2.0`. Only the
// Region 2 arm of the K*Omega^2 state machine changes; Region 1.5 (which takes
// GenBrTq) is untouched.
//
// EXPECT: EXACTLY 48 of 60 FAIL, and the 12 that pass are EXACTLY invocations
// 3..14. That is not an estimate: `vs_state` is one of the 419 compared
// fields, so the kernel's own field log partitions its 60 cases by the
// reference's arm -- 12 at state 1 and 48 at state 2 -- and the 12 agree TO
// THE CASE with the 12 hits coverage/line_coverage.json records at
// Controllers.f90:279 across all 27 scenarios. Two instruments, one number,
// and neither was designed to check the other.

// VIT Translation Scaffold
// Function: VariableSpeedControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: c300b8358d70
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-20T07:36:31Z
//
// Unit #60. `Controllers.f90:212-365` in the clean source (54dd134).
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature and the body is transcribed statement for statement.
//
// The generator-torque controller. It chooses a demanded generator torque by
// one of three control laws, applies the shutdown ramp, saturates it against
// the most stringent maximum, rate-limits it, optionally overrides it from an
// open-loop table, and writes it to `avrSWAP(47)`.
//
// SIX CALLEES, ALL SIX ALREADY TRANSLATED, AND THE ORDER OF THE CALLS IS
// LOAD-BEARING. PIController (x1 or x2), saturate (x1 or x2), ratelimit,
// interp1d (x0..2), unwrap, PIDController. Three of them carry per-instance
// state indexed by an `objInst` counter they POST-INCREMENT -- PIController
// through `objInst%instPI`, PIDController through the same counter, ratelimit
// through `objInst%instRL` -- so the sequence in which they are issued decides
// which slot of `LocalVar%piP` / `LocalVar%rlP` each one reads and writes.
// Issuing `GenBrTq` before `GenArTq` puts each one's integrator in the other's
// slot: identical on the first case of a fresh state and permanently wrong
// afterwards. None of the six is inlined (X1).
//
// WHAT THE 27 SCENARIOS SEE, read from `coverage/line_coverage.json` against
// the clean source. 23 of the 27 scenarios reach this unit; the entry line is
// hit 407,976 times. The arms:
//
//   :240  IF (VS_FBP == VS_FBP_Variable_Pitch)     407,976 -- ALWAYS TRUE
//   :242  IF (VS_ConstPower == VS_Mode_ConstPwr)   407,976 -- ALWAYS TRUE
//   :243    the `min` arm                          407,976
//   :245    the VS_RtTq*PRC_R_Torque arm                 0 hits, all 27
//   :249    the constant-pitch arm                       0 hits, all 27
//   :253  the three-way TSR test                   407,976
//   :264    the TSR PIController                   391,977 in 22 scenarios
//   :267    IF (VS_FBP == VS_FBP_Power_Overspeed)  391,977 -- NEVER TRUE
//   :272  ELSEIF (VS_ControlMode == VS_Mode_KOmega) 15,999 in ONE scenario
//   :274/:275 GenArTq / GenBrTq                     15,999
//   :278    Region 1.5                                  12
//   :280    Region 2                                15,987
//   :282..:296 Regions 2.5, 3_ConstTrq, 3_ConstPwr, 3_FBP   0 hits, all 27
//   :300  ELSE GenTq = 0                                 0 hits, all 27
//   :305  IF (SD_Trigger == 0)                     407,976
//   :306    the pass-through                       405,978
//   :308..:312 the shutdown ramp                     1,998 in ONE scenario
//   :316  saturate                                 407,976
//   :319  ratelimit                                407,976
//   :322  IF (OL_Mode > 0 .AND. Ind_GenTq > 0)     407,976 -- NEVER TRUE
//   :324..:347 the whole open-loop block                 0 hits, all 27
//   :354..:358 VS_LastGenTrq, VS_LastGenPwr, avrSWAP(47) 407,976
//   :361  IF (aviFAIL < 0)                         407,976 -- NEVER TRUE
//
// So the SIMULATION reaches one of the three control laws in 22 scenarios and a
// second in one; four of the seven `VS_State` arms, the whole open-loop block,
// the constant-pitch maximum and the error tail are outside every scenario the
// campaign runs. The differential harness is the layer that reaches them.
//
// OUTPUTS. Thirteen scalars of `LocalVar` -- VS_KOmega2_GenTq,
// VS_ConstPwr_GenTq, VS_MaxTq, GenTq, GenArTq, GenBrTq, GenTq_SD, AzUnwrapped,
// OL_Azimuth, AzError, GenTqAz, VS_LastGenTrq, VS_LastGenPwr -- plus the
// fixed-size array `AzBuffer(2)`, the nested `piP` and `rlP` compared bytewise,
// `objInst%instPI` / `objInst%instRL`, `ErrVar` (through the callees and the
// tail), and `avrSWAP(47)`.
//
// `LocalVar%GenTq` IS READ BEFORE IT IS WRITTEN ON ONE PATH: at
// `VS_ControlMode == 0` with `SD_Trigger /= 0` and `SD_Method` neither 1 nor 2,
// the reference assigns `GenTq = 0`, then `GenTq_SD` keeps its incoming value
// and `GenTq = GenTq_SD` reads it. So the unit is not idempotent and the field
// is both an input and an output.

#include "vit_types.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// The generator-torque control-mode selectors (Constants.f90:42-46).
constexpr int VS_Mode_KOmega = 1;
constexpr int VS_Mode_WSE_TSR = 2;
constexpr int VS_Mode_Power_TSR = 3;
constexpr int VS_Mode_Torque_TSR = 4;

// The constant-power flag (Constants.f90:49-50).
constexpr int VS_Mode_ConstPwr = 1;

// The fixed-blade-pitch mode selectors (Constants.f90:53-56).
constexpr int VS_FBP_Variable_Pitch = 0;
constexpr int VS_FBP_Power_Overspeed = 1;
constexpr int VS_FBP_WSE_Ref = 2;
constexpr int VS_FBP_Torque_Ref = 3;

// The torque state machine's states (Constants.f90:59-65). `VS_State_Error`
// (0) and `VS_State_PI` (7) are named in the reference's header comment and
// tested nowhere in this body, so they are not declared here: a constant the
// translation does not use is a constant a mutation operator can perturb
// without changing anything, which is a survivor with no meaning.
constexpr int VS_State_Region_1_5 = 1;
constexpr int VS_State_Region_2 = 2;
constexpr int VS_State_Region_2_5 = 3;
constexpr int VS_State_Region_3_ConstTrq = 4;
constexpr int VS_State_Region_3_ConstPwr = 5;
constexpr int VS_State_Region_3_FBP = 6;

// CHARACTER(*), PARAMETER :: RoutineName = 'VariableSpeedControl'
//
// Twenty characters, the longest `RoutineName` this campaign has translated.
// It is arithmetic rather than cosmetic: the prefix this unit writes is
// `'VariableSpeedControl:'`, TWENTY-ONE bytes, and it composes on top of
// whatever `interp1d:` (9) or `unwrap:` (7) has already prefixed. Unit #48's
// rule -- count the staged assignments on the path before calling a red on a
// CHARACTER output a translation defect -- applies here with THREE gates on the
// C++ side (interp1d's, unwrap's and this one) against ONE in the reference's
// generated bridge.
constexpr std::string_view RoutineName = "VariableSpeedControl";

// THE TWO HELPERS BELOW ARE COPIED FROM `translations/Controllers/ipc.cpp`
// (unit #53), whose own ancestry is cablecontrol.cpp -> structuralcontrol.cpp
// -> pitchsaturation.cpp -> interp1d.cpp, the unit that established the shape.
// They are not re-derived from prose (P4). The only edit is the diagnostic
// string, which names THIS unit so a reader of stderr knows which translation
// refused.
//
// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: VariableSpeedControl: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: VariableSpeedControl: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): trailing blanks only, off the field's CURRENT length.
// `find_last_not_of` rather than a hand-written backward scan, for unit #15's
// and unit #17's reason: a loop written `while (n > 0 && s[n-1] == ' ')` offers
// a `> 0` -> `>= 0` mutant that reads the byte BEFORE the buffer, which is
// undefined behaviour rather than a wrong answer and which no value comparison
// can be relied on to catch.
//
// A NEGATIVE length is the view's NOT-ALLOCATED convention, and it collapses to
// the same empty string a zero LENGTH gives -- correct for both, because the
// reference's `RoutineName//':'//TRIM(ErrMsg)` on a zero-length ErrMsg is
// exactly `'VariableSpeedControl:'`. No `== npos` branch: `find_last_not_of`
// returns `npos`, which is `SIZE_MAX`, and `npos + 1` is 0 by the defined
// wraparound of an unsigned type.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

void VariableSpeedControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                          localvariables_view_t* LocalVar, objectinstances_t* objInst,
                          errorvariables_view_t* ErrVar) {
    // ! Pre-compute generator torque values for K*Omega^2 and constant power
    // LocalVar%VS_KOmega2_GenTq = CntrPar%VS_Rgn2K*LocalVar%GenSpeedF*LocalVar%GenSpeedF
    //
    // THREE FACTORS, TWO MULTIPLICATIONS, AND THE GROUPING IS THE REFERENCE'S.
    // The Fortran is written `a*x*x`, not `a*x**2`, so it associates left to
    // right and means `(a*x)*x` -- which is exactly what the same C++ text
    // means. The campaign's `exponent-grouping` rule is about the OTHER
    // spelling: `a*x**2` means `a*(x*x)` and needs the parentheses put back.
    // Transcribing the shape gets both right without having to decide which is
    // which.
    LocalVar->VS_KOmega2_GenTq =
        CntrPar->VS_Rgn2K * LocalVar->GenSpeedF * LocalVar->GenSpeedF;

    // LocalVar%VS_ConstPwr_GenTq = (CntrPar%VS_RtPwr/(CntrPar%VS_GenEff/100.0))/LocalVar%GenSpeedF * LocalVar%PRC_R_Torque
    //
    // `/` and `*` have equal precedence and associate left to right in both
    // languages, so the parenthesised quotient is divided by `GenSpeedF` FIRST
    // and the result multiplied by `PRC_R_Torque` -- not `GenSpeedF *
    // PRC_R_Torque` computed first and divided into. The two round differently
    // and the C++ text is identical to the Fortran text, so no algebra is done
    // here.
    //
    // `100.0` is a REAL literal under `-fdefault-real-8`, so the efficiency is
    // divided as a double and not truncated as an integer.
    //
    // `GenSpeedF` can be 0.0 -- it is a filtered measurement and the harness's
    // corpus varies it freely -- and then this is a division by zero producing
    // an infinity or a NaN, in the reference exactly as here. Nothing guards it
    // in the reference and nothing guards it here (P7).
    LocalVar->VS_ConstPwr_GenTq = (CntrPar->VS_RtPwr / (CntrPar->VS_GenEff / 100.0)) /
                                  LocalVar->GenSpeedF * LocalVar->PRC_R_Torque;

    // ! Determine maximum torque saturation limit, VS_MaxTq
    // IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
    //     IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN
    //         LocalVar%VS_MaxTq = min(LocalVar%VS_ConstPwr_GenTq, CntrPar%VS_MaxTq)
    //     ELSE
    //         LocalVar%VS_MaxTq = CntrPar%VS_RtTq * LocalVar%PRC_R_Torque
    //     END IF
    // ELSE
    //     LocalVar%VS_MaxTq = CntrPar%VS_MaxTq
    // ENDIF
    //
    // `min` IS gfortran's INTRINSIC AND ITS C++ SPELLING IS `std::fmin`, not a
    // branch. Unit #21 measured this at the campaign's flags over 12,167
    // triples: `fmin`/`fmax` agree with the intrinsic on all of them, while
    // both branch spellings differ at a signed zero and at a NaN, in opposite
    // directions (`translations/Functions/saturate.cpp:20-36`).
    //
    // NOTE THE TWO DIFFERENT `VS_MaxTq`s. `LocalVar%VS_MaxTq` is what this
    // block computes; `CntrPar%VS_MaxTq` is the configured ceiling it is
    // computed FROM, and both appear in the same expression. The same name in
    // two types is the transcription error no amount of staring finds, because
    // both spellings compile.
    if (CntrPar->VS_FBP == VS_FBP_Variable_Pitch) {
        // Variable pitch mode
        if (CntrPar->VS_ConstPower == VS_Mode_ConstPwr) {
            LocalVar->VS_MaxTq = std::fmin(LocalVar->VS_ConstPwr_GenTq, CntrPar->VS_MaxTq);
        } else {
            LocalVar->VS_MaxTq = CntrPar->VS_RtTq * LocalVar->PRC_R_Torque;
        }
    } else {
        // Constant pitch, max torque is control parameter
        LocalVar->VS_MaxTq = CntrPar->VS_MaxTq;
    }

    // ! Optimal Tip-Speed-Ratio tracking controller
    // IF ((CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) .OR. \
    //     (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) .OR. \
    //     (CntrPar%VS_ControlMode == VS_Mode_Torque_TSR)) THEN
    //
    // `.OR.` IS NOT `||`. Fortran does not promise short-circuit evaluation and
    // C++ does, but all three operands here are pure integer comparisons on the
    // same field with no side effects, so the two agree on every input and the
    // only observable difference is which mutants the sweep can offer. Written
    // as `||` because that is the shape a reader of the C++ expects; the cost
    // is that a `negate_cond` on the first disjunct can be masked by the other
    // two, which is true of the Fortran as well.
    if ((CntrPar->VS_ControlMode == VS_Mode_WSE_TSR) ||
        (CntrPar->VS_ControlMode == VS_Mode_Power_TSR) ||
        (CntrPar->VS_ControlMode == VS_Mode_Torque_TSR)) {
        // ! PI controller
        // LocalVar%GenTq = PIController( &
        //                             LocalVar%VS_SpdErr, &
        //                             CntrPar%VS_KP(1), &
        //                             CntrPar%VS_KI(1), &
        //                             CntrPar%VS_MinTq, LocalVar%VS_MaxTq, &
        //                             LocalVar%DT, LocalVar%VS_LastGenTrq, LocalVar%piP, LocalVar%restart, objInst%instPI)
        //
        // CALLED, not inlined (X1). `PIController` is unit #33 and owns the
        // `piP%ITerm(inst)` / `piP%ITermLast(inst)` state writes and the
        // `instPI` post-increment.
        //
        // THE SATURATION PAIR IS MIXED: the lower bound is `CntrPar%VS_MinTq`
        // and the upper is `LocalVar%VS_MaxTq`, the value the block above just
        // computed -- not `CntrPar%VS_MaxTq`. The integrator's initial value is
        // `LocalVar%VS_LastGenTrq`, which is this unit's own output from the
        // previous call, so the loop is closed through the type rather than
        // through the callee's state on a restart.
        //
        // `LocalVar%restart` is `LOGICAL` in the reference and `int8_t` in the
        // view; the bridge takes `INTEGER(C_INT)`, so it is converted with an
        // explicit `? 1 : 0` rather than by an implicit widening that would
        // pass through whatever byte the LOGICAL holds.
        LocalVar->GenTq = picontroller_c(
            LocalVar->VS_SpdErr, CntrPar->VS_KP[0], CntrPar->VS_KI[0], CntrPar->VS_MinTq,
            LocalVar->VS_MaxTq, LocalVar->DT, LocalVar->VS_LastGenTrq, &LocalVar->piP,
            LocalVar->restart ? 1 : 0, &objInst->instPI);

        // ! Saturate control input to Region 3 constant-power value if FBP mode
        // ! is set to constant-power overspeed
        // IF (CntrPar%VS_FBP == VS_FBP_Power_Overspeed) THEN
        //     LocalVar%GenTq = MIN(LocalVar%VS_ConstPwr_GenTq, LocalVar%GenTq)
        // ENDIF
        //
        // Reached 391,977 times in the 27 scenarios and TRUE on none of them:
        // `VS_FBP` is 0 in every shipped `Examples/DISCON*.IN`. Only the
        // differential harness enters this arm.
        if (CntrPar->VS_FBP == VS_FBP_Power_Overspeed) {
            LocalVar->GenTq = std::fmin(LocalVar->VS_ConstPwr_GenTq, LocalVar->GenTq);
        }

    // ! K*Omega^2 control law with PI torque control in transition regions
    // ELSEIF (CntrPar%VS_ControlMode == VS_Mode_KOmega) THEN
    } else if (CntrPar->VS_ControlMode == VS_Mode_KOmega) {
        // ! Update PI loops for region 1.5 and 2.5 PI control
        // LocalVar%GenArTq = PIController(LocalVar%VS_SpdErrAr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MaxOMTq, CntrPar%VS_ArSatTq, LocalVar%DT, CntrPar%VS_MaxOMTq, LocalVar%piP, LocalVar%restart, objInst%instPI)
        // LocalVar%GenBrTq = PIController(LocalVar%VS_SpdErrBr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MinTq, CntrPar%VS_MinOMTq, LocalVar%DT, CntrPar%VS_MinOMTq, LocalVar%piP, LocalVar%restart, objInst%instPI)
        //
        // TWO CALLS, IN THIS ORDER, AND THE ORDER DECIDES THE SLOTS. Both read
        // and write `LocalVar%piP` through the SAME `objInst%instPI` counter,
        // which `PIController` post-increments; the "Ar" (above-rated) loop
        // takes the lower slot and the "Br" (below-rated) loop the next one.
        // They also take DIFFERENT saturation pairs and DIFFERENT initial
        // integrator values -- Ar is `[VS_MaxOMTq, VS_ArSatTq]` starting at
        // `VS_MaxOMTq`, Br is `[VS_MinTq, VS_MinOMTq]` starting at
        // `VS_MinOMTq` -- so a swap is not a relabelling.
        //
        // Note that the Ar pair's LOWER bound is `VS_MaxOMTq`, the maximum
        // torque of the optimal-mode region: above rated, the below-rated
        // maximum is the floor. That reads like a transposition and is not one.
        LocalVar->GenArTq = picontroller_c(
            LocalVar->VS_SpdErrAr, CntrPar->VS_KP[0], CntrPar->VS_KI[0], CntrPar->VS_MaxOMTq,
            CntrPar->VS_ArSatTq, LocalVar->DT, CntrPar->VS_MaxOMTq, &LocalVar->piP,
            LocalVar->restart ? 1 : 0, &objInst->instPI);
        LocalVar->GenBrTq = picontroller_c(
            LocalVar->VS_SpdErrBr, CntrPar->VS_KP[0], CntrPar->VS_KI[0], CntrPar->VS_MinTq,
            CntrPar->VS_MinOMTq, LocalVar->DT, CntrPar->VS_MinOMTq, &LocalVar->piP,
            LocalVar->restart ? 1 : 0, &objInst->instPI);

        // ! State machine if switching to blade pitch control
        // IF (LocalVar%VS_State == VS_State_Region_1_5) THEN
        //     LocalVar%GenTq = LocalVar%GenBrTq
        // ELSEIF (LocalVar%VS_State == VS_State_Region_2) THEN
        //     LocalVar%GenTq = LocalVar%VS_KOmega2_GenTq
        // ELSEIF (LocalVar%VS_State == VS_State_Region_2_5) THEN
        //     LocalVar%GenTq = LocalVar%GenArTq
        // ELSEIF (LocalVar%VS_State == VS_State_Region_3_ConstTrq) THEN
        //     LocalVar%GenTq = CntrPar%VS_RtTq
        // ELSEIF (LocalVar%VS_State == VS_State_Region_3_ConstPwr) THEN
        //     LocalVar%GenTq = LocalVar%VS_ConstPwr_GenTq
        // ELSEIF (LocalVar%VS_State == VS_State_Region_3_FBP) THEN
        //     IF (CntrPar%VS_FBP == VS_FBP_Power_Overspeed) THEN
        //         LocalVar%GenTq = MIN(LocalVar%VS_ConstPwr_GenTq, LocalVar%VS_KOmega2_GenTq)
        //     ELSEIF ((CntrPar%VS_FBP == VS_FBP_WSE_Ref) .OR. (CntrPar%VS_FBP == VS_FBP_Torque_Ref)) THEN
        //         LocalVar%GenTq = LocalVar%GenArTq
        //     ENDIF
        // END IF
        //
        // THERE IS NO FINAL `ELSE`, AND THERE ARE TWO PLACES WHERE THAT MATTERS.
        // At any `VS_State` outside 1..6 -- state 0 (`VS_State_Error`) and state
        // 7 (`VS_State_PI`) both exist in the reference's own header comment --
        // `GenTq` keeps whatever it held on entry. And inside the Region 3 FBP
        // arm, `VS_FBP == VS_FBP_Variable_Pitch` (0) falls through both inner
        // tests and leaves it alone too. Neither is written as an assignment
        // here, because the reference does not write one.
        if (LocalVar->VS_State == VS_State_Region_1_5) {
            LocalVar->GenTq = LocalVar->GenBrTq;
        } else if (LocalVar->VS_State == VS_State_Region_2) {
            // KERNEL RED TEST STUB: the Region 2 answer DOUBLED. See the header.
            LocalVar->GenTq = LocalVar->VS_KOmega2_GenTq * 2.0;
        } else if (LocalVar->VS_State == VS_State_Region_2_5) {
            LocalVar->GenTq = LocalVar->GenArTq;
        } else if (LocalVar->VS_State == VS_State_Region_3_ConstTrq) {
            LocalVar->GenTq = CntrPar->VS_RtTq;
        } else if (LocalVar->VS_State == VS_State_Region_3_ConstPwr) {
            LocalVar->GenTq = LocalVar->VS_ConstPwr_GenTq;
        } else if (LocalVar->VS_State == VS_State_Region_3_FBP) {
            // Constant power overspeed
            if (CntrPar->VS_FBP == VS_FBP_Power_Overspeed) {
                // K*Omega^2 in Region 2 or constant power overspeed in Region 3
                LocalVar->GenTq =
                    std::fmin(LocalVar->VS_ConstPwr_GenTq, LocalVar->VS_KOmega2_GenTq);
            // Reference-tracking in Region 3
            } else if ((CntrPar->VS_FBP == VS_FBP_WSE_Ref) ||
                       (CntrPar->VS_FBP == VS_FBP_Torque_Ref)) {
                LocalVar->GenTq = LocalVar->GenArTq;
            }
        }

    // ELSE        ! VS_ControlMode of 0
    //     LocalVar%GenTq = 0
    // ENDIF
    //
    // The literal is the INTEGER `0` assigned to a `REAL(DbKi)` field, which is
    // the exact double 0.0 with a positive sign. `0` rather than `0.0` here for
    // the same reason every other literal in this file is the reference's: the
    // two are the same value and the transcription is the thing being checked.
    } else {
        LocalVar->GenTq = 0;
    }

    // ! Shutdown
    // IF (LocalVar%SD_Trigger == 0) THEN
    //     LocalVar%GenTq_SD = LocalVar%GenTq
    // ELSE
    //     IF (CntrPar%SD_Method == 1 .OR. CntrPar%SD_Method == 2) THEN
    //         LocalVar%GenTq_SD = LocalVar%GenTq_SD - LocalVar%SD_MaxTorqueRate*LocalVar%DT
    //         LocalVar%GenTq_SD = saturate(LocalVar%GenTq_SD, CntrPar%VS_MinTq, CntrPar%VS_MaxTq)
    //     ENDIF
    //     LocalVar%GenTq = LocalVar%GenTq_SD
    // ENDIF
    //
    // `LocalVar%SD_MaxTorqueRate` IS THE SCALAR IN `LocalVariables`, NOT the
    // `REAL(DbKi), DIMENSION(:)` of the same name in `ControlParameters`. Both
    // exist; the reference reads the scalar, which `Shutdown` (unit #41) wrote.
    //
    // The ELSE arm with `SD_Method` neither 1 nor 2 assigns `GenTq` from a
    // `GenTq_SD` this call never wrote -- the incoming value of the field. That
    // is the path on which `GenTq` is an INPUT, and the reference has no
    // initialiser for it either.
    //
    // `saturate` is CALLED, not inlined (X1): it is unit #24 and its whole body
    // is `fmin(fmax(...))`, which is exactly the reason to call it rather than
    // to write the two intrinsics again and have two copies to keep agreeing.
    if (LocalVar->SD_Trigger == 0) {
        LocalVar->GenTq_SD = LocalVar->GenTq;
    } else {
        if (CntrPar->SD_Method == 1 || CntrPar->SD_Method == 2) {
            LocalVar->GenTq_SD =
                LocalVar->GenTq_SD - LocalVar->SD_MaxTorqueRate * LocalVar->DT;
            LocalVar->GenTq_SD =
                saturate_c(LocalVar->GenTq_SD, CntrPar->VS_MinTq, CntrPar->VS_MaxTq);
        }
        LocalVar->GenTq = LocalVar->GenTq_SD;
    }

    // ! Saturate based on most stringent defined maximum
    // LocalVar%GenTq = saturate(LocalVar%GenTq, CntrPar%VS_MinTq, MIN(CntrPar%VS_MaxTq, LocalVar%VS_MaxTq))
    //
    // THE UPPER BOUND IS THE MINIMUM OF THE TWO `VS_MaxTq`s -- the configured
    // one in `CntrPar` and the computed one in `LocalVar` -- in that operand
    // order. `saturate` itself then does `fmin(fmax(v, lo), hi)`, so with an
    // inverted pair (`lo > hi`) the answer is `hi`, which is the reference's
    // answer too.
    LocalVar->GenTq = saturate_c(LocalVar->GenTq, CntrPar->VS_MinTq,
                                 std::fmin(CntrPar->VS_MaxTq, LocalVar->VS_MaxTq));

    // ! Saturate the commanded torque using the torque rate limit
    // LocalVar%GenTq = ratelimit(LocalVar%GenTq, -CntrPar%VS_MaxRat, CntrPar%VS_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL)
    //
    // CALLED, not inlined (X1). `ratelimit` is unit #46 and owns the
    // `rlP%LastSignal(instRL)` write and the `instRL` post-increment.
    //
    // The rate bounds are SYMMETRIC and the lower one is a UNARY NEGATION of
    // the upper -- so at `VS_MaxRat == 0.0` the lower bound is negative zero,
    // which is what the reference computes and what `saturate` inside the
    // callee then sees. `0.0 - x` would give a positive zero there and is not
    // the same expression.
    //
    // `ResetValue` is OPTIONAL in the reference and is NOT passed at this call
    // site, so the bridge's presence flag is 0 and its value slot is unread.
    LocalVar->GenTq = ratelimit_c(LocalVar->GenTq, -CntrPar->VS_MaxRat, CntrPar->VS_MaxRat,
                                  LocalVar->DT, LocalVar->restart ? 1 : 0, &LocalVar->rlP,
                                  &objInst->instRL, 0, 0.0);

    // ! Open loop torque control
    // IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_GenTq > 0)) THEN
    //
    // ZERO HITS IN ALL 27 SCENARIOS for everything inside this block. Scenario
    // 24 is the one scenario that sets `OL_Mode`, and `Read_OL_Input` returns
    // on the absent `Examples/example_inputs/OL_Mode2_Input.dat`, so its DISCON
    // calls never reach the main block -- the same fact units #23, #26, #43 and
    // #44 each measured from their own side. The differential harness is the
    // only instrument that enters here.
    if ((CntrPar->OL_Mode > 0) && (CntrPar->Ind_GenTq > 0)) {
        // ! Get current OL GenTq, applies for OL_Mode 1 and 2
        // IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
        //     LocalVar%GenTq = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_GenTq,LocalVar%OL_Index,ErrVar)
        // ENDIF
        //
        // `OL_Breakpoints(1)` IS READ WITHOUT A SIZE TEST, so an empty or
        // unallocated table is an out-of-bounds read in the reference as
        // written. It is transcribed as written; `harness/ranges.toml` is where
        // an extent that would make it well-defined belongs, not here.
        //
        // NOTE THE THIRD ARGUMENT: `LocalVar%OL_Index`, not `LocalVar%Time`.
        // The azimuth call twelve lines below interpolates the SAME breakpoint
        // vector at `LocalVar%Time` instead. Two calls, two abscissae, and
        // reading the second off the first is the transcription error this
        // block invites.
        //
        // The two extents are passed independently because the C signature
        // takes them independently. `interp1d`'s own reference ABORTS on
        // `SIZE(xData) /= SIZE(yData)` while the translation continues
        // (evidence/interp1d/reference.size-mismatch-aborts.txt); at this call
        // site the Fortran cannot produce a mismatch, since `Read_OL_Input`
        // allocates every OL_ array to one row count.
        if (LocalVar->Time >= CntrPar->OL_Breakpoints[0]) {
            LocalVar->GenTq =
                interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                           CntrPar->OL_GenTq, CntrPar->n_OL_GenTq, LocalVar->OL_Index, ErrVar);
        }

        // ! Azimuth tracking control
        // IF (CntrPar%OL_Mode == 2) THEN
        if (CntrPar->OL_Mode == 2) {
            // ! Push, pop and unwrap azimuth buffer
            // ! Initialize
            // IF (LocalVar%iStatus == 0) THEN
            //     LocalVar%AzBuffer(1) = LocalVar%Azimuth
            //     LocalVar%AzBuffer(2) = LocalVar%Azimuth
            // ENDIF
            // LocalVar%AzBuffer(1) = LocalVar%AzBuffer(2)
            // LocalVar%AzBuffer(2) = LocalVar%Azimuth
            //
            // THE INITIALISATION BLOCK IS REDUNDANT WITH THE TWO STATEMENTS
            // BELOW IT AND IS STILL NOT A NO-OP: on the `iStatus == 0` call it
            // sets element 1 to `Azimuth` and then the shift immediately
            // overwrites element 1 with element 2, which the block also set to
            // `Azimuth`. So the two arms agree on element 1 by construction and
            // differ on nothing -- unit #46's shape, an initialisation arm that
            // computes the same answer as the arm beside it. Transcribed
            // anyway, because deleting it would be the translation deciding a
            // question the reference answers in code.
            if (LocalVar->iStatus == 0) {
                LocalVar->AzBuffer[1 - 1] = LocalVar->Azimuth;
                LocalVar->AzBuffer[2 - 1] = LocalVar->Azimuth;
            }
            LocalVar->AzBuffer[1 - 1] = LocalVar->AzBuffer[2 - 1];
            LocalVar->AzBuffer[2 - 1] = LocalVar->Azimuth;

            // LocalVar%AzBuffer = UNWRAP(LocalVar%AzBuffer, ErrVar)
            //
            // CALLED, not inlined (X1). `unwrap` is unit #26 and its C entry
            // point writes its result through a fourth argument.
            //
            // THE RESULT GOES TO A TEMPORARY AND IS THEN ASSIGNED. The Fortran
            // is a function whose result is an array of `SIZE(x)`; the
            // assignment copies that temporary back over the argument. Passing
            // `AzBuffer` as both the input and the output would happen to give
            // the same answer -- `unwrap` copies `x` into `y` before it touches
            // anything -- but it would rest on a property of the callee rather
            // than on the reference's own semantics, and the callee is free to
            // change. Two doubles of stack is the price of not depending on it.
            //
            // `SIZE(x)` is 2: `REAL(DbKi) :: AzBuffer(2)` (ROSCO_Types.f90:333).
            double AzBuffer_unwrapped[2];
            unwrap_c(LocalVar->AzBuffer, 2, ErrVar, AzBuffer_unwrapped);
            LocalVar->AzBuffer[1 - 1] = AzBuffer_unwrapped[1 - 1];
            LocalVar->AzBuffer[2 - 1] = AzBuffer_unwrapped[2 - 1];

            // LocalVar%AzUnwrapped = LocalVar%AzBuffer(2)
            LocalVar->AzUnwrapped = LocalVar->AzBuffer[2 - 1];

            // ! Current desired Azimuth, error
            // LocalVar%OL_Azimuth = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_Azimuth,LocalVar%Time,ErrVar)
            // LocalVar%AzError = LocalVar%OL_Azimuth - LocalVar%AzUnwrapped
            //
            // `CntrPar%OL_Azimuth` (the table) and `LocalVar%OL_Azimuth` (the
            // scalar this statement writes) are two different fields with the
            // same name in two different types, in the SAME statement. The
            // abscissa is `LocalVar%Time` here, not `LocalVar%OL_Index`.
            LocalVar->OL_Azimuth =
                interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                           CntrPar->OL_Azimuth, CntrPar->n_OL_Azimuth, LocalVar->Time, ErrVar);
            LocalVar->AzError = LocalVar->OL_Azimuth - LocalVar->AzUnwrapped;

            // LocalVar%GenTqAz = PIDController(LocalVar%AzError, CntrPar%RP_Gains(1), CntrPar%RP_Gains(2), CntrPar%RP_Gains(3), CntrPar%RP_Gains(4), -LocalVar%VS_MaxTq * 2, LocalVar%VS_MaxTq * 2, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst, LocalVar)
            // LocalVar%GenTq = LocalVar%GenTq + LocalVar%GenTqAz
            //
            // CALLED, not inlined (X1). `PIDController` is unit #34; it takes
            // the WHOLE `objInst` and the WHOLE `LocalVar` because its
            // derivative term runs a filter that needs both.
            //
            // FOUR GAINS OUT OF ONE ARRAY, IN ORDER: kp, ki, kd, tf.
            //
            // `-LocalVar%VS_MaxTq * 2` is unary minus binding tighter than the
            // multiply in both languages, so it is `(-VS_MaxTq) * 2`. The
            // saturation pair is therefore symmetric about zero at twice the
            // computed maximum, and at `VS_MaxTq == 0.0` the lower bound is
            // negative zero.
            //
            // `LocalVar%piP` is passed here AND to the `PIController` call at
            // the top of the unit, through the same `instPI` counter that
            // `PIDController` post-increments -- so on a `VS_ControlMode` in
            // {2,3,4} with `OL_Mode == 2` this call takes the slot AFTER the
            // torque loop's.
            LocalVar->GenTqAz = pidcontroller_c(
                LocalVar->AzError, CntrPar->RP_Gains[0], CntrPar->RP_Gains[1],
                CntrPar->RP_Gains[2], CntrPar->RP_Gains[3], -LocalVar->VS_MaxTq * 2,
                LocalVar->VS_MaxTq * 2, LocalVar->DT, 0.0, &LocalVar->piP,
                LocalVar->restart ? 1 : 0, objInst, LocalVar);
            LocalVar->GenTq = LocalVar->GenTq + LocalVar->GenTqAz;
        }
    }

    // ! Reset the value of LocalVar%VS_LastGenTrq to the current values:
    // LocalVar%VS_LastGenTrq = LocalVar%GenTq
    // LocalVar%VS_LastGenPwr = LocalVar%VS_GenPwr
    //
    // `VS_LastGenTrq` is the integrator seed the TSR `PIController` call at the
    // top of the NEXT invocation reads, so this statement closes the unit's own
    // feedback loop through the type. `VS_LastGenPwr` is a plain carry-forward
    // of a field this unit never computes.
    LocalVar->VS_LastGenTrq = LocalVar->GenTq;
    LocalVar->VS_LastGenPwr = LocalVar->VS_GenPwr;

    // ! Set the command generator torque (See Appendix A of Bladed User's Guide):
    // avrSWAP(47) = MAX(0.0_DbKi, LocalVar%VS_LastGenTrq)  ! Demanded generator torque, prevent negatives.
    //
    // THE ASSIGNMENT NARROWS. `avrSWAP` is `REAL(ReKi)` = REAL(4) and the
    // right-hand side is REAL(8), so gfortran rounds to single precision on the
    // store; the `static_cast<float>` is that rounding written down. Unit #44
    // established the spelling on `avrSWAP(48)`.
    //
    // The MAX is computed in DOUBLE and narrowed afterwards, which is what the
    // Fortran does -- `MAX(0.0_DbKi, x)` has a REAL(8) result and the
    // conversion happens at the assignment. `0.0_DbKi` is the FIRST operand, so
    // at `x` a negative zero the intrinsic returns the first operand's positive
    // zero; `fmax` in the same operand order does the same.
    avrSWAP[47 - 1] = static_cast<float>(std::fmax(0.0, LocalVar->VS_LastGenTrq));

    // ! Add RoutineName to error message
    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    // ENDIF
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
