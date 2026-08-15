// VIT Translation Scaffold
// Function: Shutdown
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE Shutdown(LocalVar, CntrPar, objInst, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 84dc2630e515
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T05:01:55Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature; the body is transcribed statement for statement.
//
// WHAT THE SIMULATION CAN SEE OF THIS UNIT, read from
// coverage/line_coverage.json against the coverage-era source (Shutdown at
// ControllerBlocks.f90:638-760) and stated here BEFORE any green is taken, so
// that a reader knows which arms a kernel or gate result is about:
//
//     :655 IF (iStatus == 0)                 11,999 calls / arm taken ONCE
//     :671 the trigger window                11,999 / the four tests 10,002
//     :683 SD_Trigger = 4  (SD_EnableTime)        1  <- the ONLY trigger fired
//     :674 / :677 / :680  Trigger = 1 / 2 / 3     0  <- pitch, yaw, genspeed
//     :688 SD_Method == 1                    11,999 <- always
//     :691 SD_Stage == 0    10,002    |  :702 the stage arm      1,997
//     :708 SD_Stage = SD_Stage + 1            0  <- never advances
//     :711 the ELSE (Stage > SD_Stage_N)      0
//     :713-:748 the whole SD_Method == 2 arm  0
//
// And scenario 9 is the only one of the 27 that reaches ANY of it: the call
// site DISCON.F90:107 sits behind `IF (CntrPar%SD_Mode > 0)` and carries hits
// under scenario 9 alone. So more than half of this unit's statements are
// outside every simulation the campaign runs, and the differential harness --
// which draws `SD_Method`, the four `SD_Enable*` flags and `SD_Stage` freely --
// is the only instrument that can reach them. Where each arm is actually
// measured is recorded in evidence/Shutdown/, not asserted here.

#include "vit_types.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <cstdlib>

// ---- PROBE ONLY. Counters and an atexit dump; no behaviour changes, so this
// ---- build runs GREEN and is a READING of the corpus rather than a
// ---- perturbation of it. Not the shipped translation.
namespace {
struct VitErrMsgCensus {
    long trim_calls = 0, assign_calls = 0;
    long n_eq_1 = 0, n_le_0 = 0, size_eq_cap = 0, trailing_blank = 0;
    int n_min = 2147483647, n_max = -2147483647;
    int cap_min = 2147483647, cap_max = -2147483647;
    long msg_max = 0;
    ~VitErrMsgCensus() {
        std::fprintf(stderr,
          "VITCENSUS errmsg_trim calls %ld | assign_errmsg calls %ld\n"
          "VITCENSUS n_ErrMsg min %d max %d | n_ErrMsg_cap min %d max %d\n"
          "VITCENSUS n_ErrMsg == 1: %ld | n_ErrMsg <= 0: %ld\n"
          "VITCENSUS s.size() == n_ErrMsg_cap: %ld | longest message %ld\n"
          "VITCENSUS trailing blank inside n_ErrMsg: %ld\n",
          trim_calls, assign_calls, n_min, n_max, cap_min, cap_max,
          n_eq_1, n_le_0, size_eq_cap, msg_max, trailing_blank);
    }
};
VitErrMsgCensus vit_census;
}


namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'Shutdown'
constexpr std::string_view RoutineName = "Shutdown";

// The Fortran reads `R2D`/`D2R` from the Constants module by name. Restated
// here to the digit, as `prefiltermeasuredsignals.cpp` and `readavrswap.cpp`
// already do: the reference's own literals, not a recomputed 180/Pi, because
// 57.2957795130 and 0.01745329251 are TRUNCATED decimals and not the correctly
// rounded conversions -- 180/Pi is 57.29577951308232, and the difference is
// visible in the last places of every angle this unit filters.
constexpr double R2D = 57.2957795130;
constexpr double D2R = 0.01745329251;

// THE TWO HELPERS BELOW ARE COPIED FROM `translations/ControllerBlocks/
// pitchsaturation.cpp` -- the unit in this module that established the shape,
// which took them in turn from `interp1d.cpp` -- with the literal name in the
// two diagnostic strings changed to this unit's. They are not re-derived from
// the prose (P4).

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one.
//
// The capacity test is reachable in principle: the message written is nine
// characters longer than the one it was handed (`Shutdown` plus the colon), so
// an `ErrMsg` arriving within nine bytes of the buffer's capacity overflows it.
// Which inputs reach it is a question for the corpus and not for this comment.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    ++vit_census.assign_calls;
    if (ErrVar->n_ErrMsg_cap < vit_census.cap_min) vit_census.cap_min = ErrVar->n_ErrMsg_cap;
    if (ErrVar->n_ErrMsg_cap > vit_census.cap_max) vit_census.cap_max = ErrVar->n_ErrMsg_cap;
    if (static_cast<long>(s.size()) > vit_census.msg_max) vit_census.msg_max = static_cast<long>(s.size());
    if (static_cast<int>(s.size()) == ErrVar->n_ErrMsg_cap) ++vit_census.size_eq_cap;
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: Shutdown: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: Shutdown: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): trailing blanks only, off the field's CURRENT length.
// `find_last_not_of` rather than a hand-written backward scan: a loop written
// `while (n > 0 && s[n-1] == ' ')` offers a `> 0` -> `>= 0` mutant that reads
// the byte BEFORE the buffer, which is undefined behaviour rather than a wrong
// answer and which no value comparison can be relied on to catch.
//
// A NEGATIVE length is the view's NOT-ALLOCATED convention and collapses to the
// same empty string a zero LENGTH gives, which is correct for both.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    ++vit_census.trim_calls;
    if (n < vit_census.n_min) vit_census.n_min = n;
    if (n > vit_census.n_max) vit_census.n_max = n;
    if (n == 1) ++vit_census.n_eq_1;
    if (n <= 0) ++vit_census.n_le_0;
    if (n > 0 && ErrVar->ErrMsg != nullptr && ErrVar->ErrMsg[n - 1] == ' ') ++vit_census.trailing_blank;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

void Shutdown(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
              objectinstances_t* objInst, errorvariables_view_t* ErrVar) {
    // REAL(DbKi) :: SD_NacVaneCosF, SD_NacVaneSinF
    // INTEGER(IntKi) :: I_Stage
    //
    // The two filtered vane components are LOCALS in the reference and are
    // locals here. Only their combination reaches `LocalVar%SD_NacVaneF`, so
    // neither is separately observable by any instrument.
    double SD_NacVaneCosF;
    double SD_NacVaneSinF;

    // !Initialize shutdown trigger variable
    // IF (LocalVar%iStatus == 0) THEN
    //     LocalVar%SD_Trigger = 0
    // ENDIF
    if (LocalVar->iStatus == 0) {
        LocalVar->SD_Trigger = 0;
    }

    // ! Filter pitch signal
    // LocalVar%SD_BlPitchF = LPFilter(LocalVar%BlPitchCMeas, LocalVar%DT,
    //     CntrPar%SD_PitchCornerFreq, LocalVar%FP, LocalVar%iStatus,
    //     LocalVar%restart, objInst%instLPF)
    // ! Filter generator speed
    // LocalVar%SD_GenSpeedF = LPFilter(LocalVar%Genspeed, LocalVar%DT,
    //     CntrPar%SD_GenSpdCornerFreq, ...)
    //
    // FOUR LPFilter CALLS SHARE ONE COUNTER. `objInst%instLPF` is INOUT and
    // LPFilter post-increments it, so each of these four occupies a distinct
    // slot in the filter's DIMENSION(1024) state arrays and THE ORDER OF THE
    // FOUR CALLS IS LOAD-BEARING. Swapping any two would keep every argument
    // correct and silently exchange two filters' histories.
    //
    // The reference's optional `InitialValue` is absent at all four sites, so
    // `has_InitialValue` is 0 and the value slot is unread. `LocalVar%restart`
    // is LOGICAL in the reference and `int8_t` in the view; `? 1 : 0` is the
    // spelling every other unit in this campaign uses for it.
    //
    // Unlike PreFilterMeasuredSignals -- whose two vane filters pass the
    // literal `.FALSE.` -- ALL FOUR of this unit's calls pass
    // `LocalVar%restart`. The distinction is worth stating because the two
    // units' vane blocks are otherwise the same three lines.
    LocalVar->SD_BlPitchF =
        lpfilter_c(LocalVar->BlPitchCMeas, LocalVar->DT, CntrPar->SD_PitchCornerFreq, &LocalVar->FP,
                   LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
    LocalVar->SD_GenSpeedF =
        lpfilter_c(LocalVar->GenSpeed, LocalVar->DT, CntrPar->SD_GenSpdCornerFreq, &LocalVar->FP,
                   LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);

    // ! Filter yaw error signal (NacVane)
    // SD_NacVaneCosF = LPFilter(cos(LocalVar%NacVane*D2R), ...)
    // SD_NacVaneSinF = LPFilter(sin(LocalVar%NacVane*D2R), ...)
    // LocalVar%SD_NacVaneF = wrap_180(atan2(SD_NacVaneSinF, SD_NacVaneCosF) * R2D)
    //
    // COS first, then SIN -- that is the reference's order and it decides which
    // of the two gets the lower `instLPF` slot. `atan2` takes (y, x) in both
    // languages, so the SIN component stays first in the call.
    SD_NacVaneCosF = lpfilter_c(std::cos(LocalVar->NacVane * D2R), LocalVar->DT,
                                CntrPar->SD_YawErrorCornerFreq, &LocalVar->FP, LocalVar->iStatus,
                                LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
    SD_NacVaneSinF = lpfilter_c(std::sin(LocalVar->NacVane * D2R), LocalVar->DT,
                                CntrPar->SD_YawErrorCornerFreq, &LocalVar->FP, LocalVar->iStatus,
                                LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
    LocalVar->SD_NacVaneF = wrap_180_c(std::atan2(SD_NacVaneSinF, SD_NacVaneCosF) * R2D);

    // ! See if we should shutdown
    // IF ((LocalVar%SD_Trigger == 0) .AND. (LocalVar%Time >= CntrPar%SD_TimeActivate)) THEN
    //
    // FOUR INDEPENDENT `IF`s, NOT AN `ELSEIF` CHAIN. They run in order and each
    // overwrites the last, so when more than one condition holds the LAST one
    // wins: time (4) beats generator speed (3) beats yaw (2) beats pitch (1).
    // A chain would be a different program and would agree with this one on
    // every input where at most one condition is true -- which is every input
    // scenario 9 produces.
    if ((LocalVar->SD_Trigger == 0) && (LocalVar->Time >= CntrPar->SD_TimeActivate)) {
        // IF (CntrPar%SD_EnablePitch==1 .AND. LocalVar%SD_BlPitchF > CntrPar%SD_MaxPit) THEN
        //     LocalVar%SD_Trigger = 1
        if ((CntrPar->SD_EnablePitch == 1) && (LocalVar->SD_BlPitchF > CntrPar->SD_MaxPit)) {
            LocalVar->SD_Trigger = 1;
        }
        // IF (CntrPar%SD_EnableYawError==1 .AND. ABS(LocalVar%SD_NacVaneF) > CntrPar%SD_MaxYawError)
        //     LocalVar%SD_Trigger = 2
        //
        // `std::fabs` on a double: Fortran's ABS for REAL(8) clears the sign
        // bit, which is what fabs does, including on the signed zeros and NaN.
        if ((CntrPar->SD_EnableYawError == 1) &&
            (std::fabs(LocalVar->SD_NacVaneF) > CntrPar->SD_MaxYawError)) {
            LocalVar->SD_Trigger = 2;
        }
        // IF (CntrPar%SD_EnableGenSpeed==1 .AND. LocalVar%SD_GenSpeedF > CntrPar%SD_MaxGenSpd) THEN
        //     LocalVar%SD_Trigger = 3
        if ((CntrPar->SD_EnableGenSpeed == 1) && (LocalVar->SD_GenSpeedF > CntrPar->SD_MaxGenSpd)) {
            LocalVar->SD_Trigger = 3;
        }
        // IF (CntrPar%SD_EnableTime==1 .AND. LocalVar%Time > CntrPar%SD_Time) THEN
        //     LocalVar%SD_Trigger = 4
        //
        // Note the STRICT `>` here against the `>=` on `SD_TimeActivate` in the
        // enclosing test. The two are three lines apart and read the same
        // clock; transcribed as written rather than made consistent.
        if ((CntrPar->SD_EnableTime == 1) && (LocalVar->Time > CntrPar->SD_Time)) {
            LocalVar->SD_Trigger = 4;
        }
    }

    // ! Method 1: stage depends on time
    // IF (CntrPar%SD_Method == 1) THEN
    if (CntrPar->SD_Method == 1) {
        // IF (LocalVar%SD_Stage == 0) THEN
        if (LocalVar->SD_Stage == 0) {
            // ! Normal operation, check for shutdown trigger
            // IF (LocalVar%SD_Trigger > 0) THEN
            //     LocalVar%SD_Stage = 1
            //     LocalVar%SD_StageStartTime = LocalVar%Time
            // ENDIF
            if (LocalVar->SD_Trigger > 0) {
                LocalVar->SD_Stage = 1;
                LocalVar->SD_StageStartTime = LocalVar->Time;
            }

            // ! Set maximum pitch and torque rates (for completeness)
            // LocalVar%SD_MaxPitchRate  = 0
            // LocalVar%SD_MaxTorqueRate = 0
            //
            // These two run whether or not the trigger fired -- they are OUTSIDE
            // the `IF` above, so on the very call that sets SD_Stage = 1 the
            // rates are still zeroed.
            LocalVar->SD_MaxPitchRate = 0;
            LocalVar->SD_MaxTorqueRate = 0;

            // ELSEIF (LocalVar%SD_Stage .LE. CntrPar%SD_Stage_N) THEN
            //
            // The reference's comment on the following ELSE calls this arm
            // "Stage > 0", but the CONDITION is only `SD_Stage /= 0`. A negative
            // `SD_Stage` therefore lands here and subscripts the two allocatable
            // rate arrays out of bounds. That is the reference's behaviour, not
            // a translation choice; it is pinned in harness/ranges.toml rather
            // than guarded here, because a guard would be a second program.
        } else if (LocalVar->SD_Stage <= CntrPar->SD_Stage_N) {
            // LocalVar%SD_MaxPitchRate  = CntrPar%SD_MaxPitchRate(LocalVar%SD_Stage)
            // LocalVar%SD_MaxTorqueRate = CntrPar%SD_MaxTorqueRate(LocalVar%SD_Stage)
            //
            // 1-based Fortran subscript on a rank-1 allocatable -> `[i - 1]`.
            LocalVar->SD_MaxPitchRate = CntrPar->SD_MaxPitchRate[LocalVar->SD_Stage - 1];
            LocalVar->SD_MaxTorqueRate = CntrPar->SD_MaxTorqueRate[LocalVar->SD_Stage - 1];

            // ! Shutdown stage
            // IF (LocalVar%Time >= LocalVar%SD_StageStartTime
            //                      + CntrPar%SD_StageTime(LocalVar%SD_Stage)) THEN
            //     LocalVar%SD_Stage = LocalVar%SD_Stage + 1
            //     LocalVar%SD_StageStartTime = LocalVar%Time
            // ENDIF
            //
            // The sum is formed as the reference forms it -- start time plus the
            // stage's duration -- and compared. Rewriting it as
            // `Time - StageStartTime >= StageTime` is the same in real
            // arithmetic and a different rounding, so it is not done.
            //
            // `SD_StageTime` is subscripted with the CURRENT stage, before the
            // increment; the increment can carry `SD_Stage` past `SD_Stage_N`,
            // which is what the third arm exists to catch on the NEXT call.
            if (LocalVar->Time >=
                LocalVar->SD_StageStartTime + CntrPar->SD_StageTime[LocalVar->SD_Stage - 1]) {
                LocalVar->SD_Stage = LocalVar->SD_Stage + 1;
                LocalVar->SD_StageStartTime = LocalVar->Time;
            }

            // ELSE  ! Stage > CntrPar%SD_Stage_N
            //     LocalVar%SD_MaxPitchRate  = CntrPar%PC_MaxRat
            //     LocalVar%SD_MaxTorqueRate = CntrPar%VS_MaxRat
        } else {
            LocalVar->SD_MaxPitchRate = CntrPar->PC_MaxRat;
            LocalVar->SD_MaxTorqueRate = CntrPar->VS_MaxRat;
        }

        // ! Shutdown method 2: stage depends on blade pitch (LocalVar%BlPitchCMeas)
        // ELSEIF (CntrPar%SD_Method == 2) THEN
        //
        // NO FINAL `ELSE`. With `SD_Method` neither 1 nor 2 the unit leaves
        // `SD_Stage`, `SD_MaxPitchRate` and `SD_MaxTorqueRate` exactly as it
        // found them -- it does not zero them. That absence is part of the
        // contract and is transcribed as an absence (P6).
    } else if (CntrPar->SD_Method == 2) {
        // IF (LocalVar%SD_Stage == 0) THEN
        if (LocalVar->SD_Stage == 0) {
            // IF (LocalVar%SD_Trigger > 0) THEN
            //     LocalVar%SD_Stage = 1
            // ENDIF
            //
            // METHOD 2 DOES NOT SET `SD_StageStartTime` HERE and method 1 does.
            // The stage clock is unused in this method, so the field keeps
            // whatever the caller left in it.
            if (LocalVar->SD_Trigger > 0) {
                LocalVar->SD_Stage = 1;
            }

            // LocalVar%SD_MaxPitchRate  = 0
            // LocalVar%SD_MaxTorqueRate = 0
            LocalVar->SD_MaxPitchRate = 0;
            LocalVar->SD_MaxTorqueRate = 0;

            // ELSE ! Stage > 0
        } else {
            // ! Figure out what stage we are in
            // DO I_Stage = 1, CntrPar%SD_Stage_N
            //     IF (LocalVar%BlPitchCMeas >= CntrPar%SD_StagePitch(I_Stage)) THEN
            //         LocalVar%SD_Stage = I_Stage + 1
            //     ENDIF
            // ENDDO
            //
            // NO EARLY EXIT. The loop runs to `SD_Stage_N` and the LAST index
            // whose threshold is met wins, so a non-monotonic `SD_StagePitch`
            // gives a different answer from "the first threshold not met".
            // Transcribed as the loop it is.
            //
            // The loop counter is 1-based to match the reference's own bounds;
            // only the subscript is shifted.
            for (int I_Stage = 1; I_Stage <= CntrPar->SD_Stage_N; ++I_Stage) {
                if (LocalVar->BlPitchCMeas >= CntrPar->SD_StagePitch[I_Stage - 1]) {
                    LocalVar->SD_Stage = I_Stage + 1;
                }
            }

            // ! Set maximum pitch and torque rates
            // IF (LocalVar%SD_Stage > CntrPar%SD_Stage_N) THEN
            //     LocalVar%SD_MaxPitchRate  = CntrPar%PC_MaxRat
            //     LocalVar%SD_MaxTorqueRate = CntrPar%VS_MaxRat
            // ELSE
            //     LocalVar%SD_MaxPitchRate  = CntrPar%SD_MaxPitchRate(LocalVar%SD_Stage)
            //     LocalVar%SD_MaxTorqueRate = CntrPar%SD_MaxTorqueRate(LocalVar%SD_Stage)
            // ENDIF
            //
            // Same out-of-bounds shape as method 1's second arm: reaching the
            // ELSE with a negative `SD_Stage` subscripts before the array's
            // start. Pinned in harness/ranges.toml, not guarded.
            if (LocalVar->SD_Stage > CntrPar->SD_Stage_N) {
                LocalVar->SD_MaxPitchRate = CntrPar->PC_MaxRat;
                LocalVar->SD_MaxTorqueRate = CntrPar->VS_MaxRat;
            } else {
                LocalVar->SD_MaxPitchRate = CntrPar->SD_MaxPitchRate[LocalVar->SD_Stage - 1];
                LocalVar->SD_MaxTorqueRate = CntrPar->SD_MaxTorqueRate[LocalVar->SD_Stage - 1];
            }
        }
    }

    // ! Add RoutineName to error message
    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    // ENDIF
    //
    // This unit sets `aviFAIL` nowhere and neither of its two callees does, so
    // the branch is reached only when the CALLER arrives with it already
    // negative. coverage/line_coverage.json records ControllerBlocks.f90:757
    // at 0 hits in all 27 scenarios; the differential harness draws
    // `ErrVar%aviFAIL` and is what reaches it.
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
