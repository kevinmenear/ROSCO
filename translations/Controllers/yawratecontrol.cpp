// VIT Translation Scaffold
// Function: YawRateControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 9ebb1ecfc720
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T11:11:22Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// THIS UNIT CARRIES STATE THE SIGNATURE DOES NOT. Four of its locals are
// declared `SAVE`, and they are the campaign's first SAVE locals to cross into
// C++ as such -- unit #5's `ExtController` moved its one SAVE local to a
// Fortran bridge module instead, because `TYPE(ExtDLL_Type)` has no C
// representation worth mirroring. These four are scalars and do:
//
//     REAL(DbKi), SAVE :: NacVaneOffset     written on every call before it is
//                                           read; SAVE not load-bearing
//     INTEGER,    SAVE :: YawState          READ AT :433 BEFORE ANY WRITE on
//                                           every call but the iStatus == 0
//                                           one -- this is the state machine
//     REAL(DbKi), SAVE :: NacHeadingError   written on every call before it is
//                                           read; SAVE not load-bearing
//     INTEGER,    SAVE :: Tidx              written at :405 and READ NOWHERE,
//                                           in this procedure or any other
//
// All four become function-local `static`, which is what VIT's own check
// registry states the construct maps to and what gfortran compiles SAVE into:
// one instance for the whole program, zero-initialised in .bss. Only `YawState`
// needs it -- but the reference declares four and the contract is mirror, so
// four are declared. `Tidx` is written and voided; deleting it would be a
// statement the reference has that this does not.
//
// THREE ARMS ARE OUTSIDE EVERY SIMULATION THE CAMPAIGN RUNS. Read from
// coverage/line_coverage.json against the clean source, all 27 scenarios
// (`YawRateControl` at Controllers.f90:367-485 at 54dd134):
//
//   :397  IF (Y_ControlMode == 1)        23,999 under 7 and 27, 15,999 under 3
//   :404/:405 the iStatus == 0 init           1 per run
//   :409  IF (ZMQ_Mode == 1)             tested on every call
//   :410  NacVaneOffset = ZMQ_YawOffset       0 hits, ALL 27 SCENARIOS
//   :426  IF (WE_Vw_F <= Y_uSwitch)      tested on every call
//   :427  deadband = Y_ErrThresh(1)           0 hits, ALL 27 SCENARIOS
//   :433/:444 the two state guards       23,999 / 13,726
//   :436-:461 all four edges and both persist arms   12 / 11 / 12 / 12 and
//                                                    10,261 / 10,215
//   :472  IF (OL_Mode > 0 .AND. Ind_YawRate > 0)   tested on every call
//   :473-:475 the open-loop override          0 hits, ALL 27 SCENARIOS
//
// So the kernel and the gate can see the whole state machine -- every arm, and
// each of the four edges at the invocation vit.yaml's window was aimed at --
// and they cannot see the ZMQ offset, the low-wind deadband, or the open-loop
// override, which is this unit's only call to `interp1d`. The third is dead for
// the reason units #23, #26 and #43 measured and STATUS.md has carried since
// phase 3: scenario 24 is the one scenario that sets `OL_Mode`, and
// `Read_OL_Input` returns on the absent `OL_Mode2_Input.dat`, so its DISCON
// calls never reach the main block. The differential harness draws `ZMQ_Mode`,
// `Y_uSwitch`, `OL_Mode`, `Ind_YawRate` and the two OL arrays freely and is the
// instrument that reaches all three; where that is measured is recorded in
// evidence/YawRateControl/, not asserted here.

#include "vit_types.h"

#include <cmath>

namespace {

// The Fortran reads `R2D`/`D2R` from the Constants module by name. Restated
// here rather than recomputed, and the distinction is load-bearing:
// 57.2957795130 and 0.01745329251 are TRUNCATED decimals and not the correctly
// rounded conversions -- 180/Pi is 57.29577951308232 -- so `180.0 / M_PI` is a
// different number. Copied character for character from
// `rosco/controller/src/Constants.f90:22-23`, the same way
// `translations/ControllerBlocks/shutdown.cpp` and
// `translations/Filters/prefiltermeasuredsignals.cpp` restate them.
constexpr double R2D = 57.2957795130;
constexpr double D2R = 0.01745329251;

}  // namespace

void YawRateControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                    localvariables_view_t* LocalVar, objectinstances_t* objInst,
                    debugvariables_t* DebugVar, errorvariables_view_t* ErrVar) {
    // REAL(DbKi), SAVE :: NacVaneOffset
    // INTEGER,    SAVE :: YawState
    // REAL(DbKi), SAVE :: NacHeadingError
    // INTEGER,    SAVE :: Tidx
    //
    // Function-local statics: one instance for the whole program, zero at
    // program start. gfortran gives an un-initialised SAVE the same .bss zero,
    // so a first call arriving with `iStatus /= 0` -- which skips the init arm
    // -- reads 0 on both sides rather than reading two different kinds of
    // nothing.
    static double NacVaneOffset;
    static int YawState;
    static double NacHeadingError;
    static int Tidx;

    // REAL(DbKi) :: WindDirPlusOffset, WindDirPlusOffsetCosF,
    //               WindDirPlusOffsetSinF, NacHeadingTarget, YawRateCom, deadband
    //
    // `REAL(DbKi) :: Time` is declared by the reference and never used; it is
    // not restated, because an unused local is not a statement.
    double WindDirPlusOffset;
    double WindDirPlusOffsetCosF;
    double WindDirPlusOffsetSinF;
    double NacHeadingTarget;
    double YawRateCom;
    double deadband;

    // IF (CntrPar%Y_ControlMode == 1) THEN
    //
    // The whole body is inside this test, and there is no ELSE: at any other
    // `Y_ControlMode` -- 0 being what every scenario that does not enable yaw
    // control holds it at -- this procedure writes nothing at all, not even
    // `avrSWAP(48)`, and the five DebugVar fields keep whatever they held.
    if (CntrPar->Y_ControlMode == 1) {

        // ! Compass wind directions in degrees
        // LocalVar%WindDir = wrap_180(LocalVar%NacHeading + LocalVar%NacVane)
        LocalVar->WindDir = wrap_180_c(LocalVar->NacHeading + LocalVar->NacVane);

        // ! Initialize
        // IF (LocalVar%iStatus == 0) THEN
        //     YawState = 0
        //     Tidx = 1
        // ENDIF
        //
        // The ONLY write to `Tidx` in the whole of ROSCO. Nothing reads it --
        // not this procedure, not any other, and its comment describes an
        // interpolation index for a feature the shipped source does not have.
        // Written because the reference writes it (P7); voided immediately
        // below so that the store is not elided into a warning, and so that the
        // day something reads it the void goes away with the read.
        if (LocalVar->iStatus == 0) {
            YawState = 0;
            Tidx = 1;
        }
        (void)Tidx;

        // ! Compute/apply offset
        // IF (CntrPar%ZMQ_Mode == 1) THEN
        //     NacVaneOffset = LocalVar%ZMQ_YawOffset
        // ELSE
        //     NacVaneOffset = CntrPar%Y_MErrSet ! (deg) # Offset from setpoint
        // ENDIF
        if (CntrPar->ZMQ_Mode == 1) {
            NacVaneOffset = LocalVar->ZMQ_YawOffset;
        } else {
            NacVaneOffset = CntrPar->Y_MErrSet;
        }

        // ! Update filtered wind direction
        // WindDirPlusOffset = wrap_180(LocalVar%WindDir + NacVaneOffset)
        // WindDirPlusOffsetCosF = LPFilter(cos(WindDirPlusOffset*D2R), LocalVar%DT,
        //     CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF)
        // WindDirPlusOffsetSinF = LPFilter(sin(WindDirPlusOffset*D2R), LocalVar%DT,
        //     CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF)
        // NacHeadingTarget = wrap_180(atan2(WindDirPlusOffsetSinF, WindDirPlusOffsetCosF) * R2D)
        //
        // COS first, then SIN -- that is the reference's order and it decides
        // which of the two gets the lower `instLPF` slot, since `LPFilter`
        // post-increments the instance counter. `atan2` takes (y, x) in both
        // languages, so the SIN component stays first in that call. Same shape
        // as `shutdown.cpp` and `prefiltermeasuredsignals.cpp`, which filter
        // the same signal through the same corner frequency.
        //
        // The sixth argument is the literal `.FALSE.`, not a variable: this
        // call site never asks LPFilter to reset. `lpfilter_c` takes it as
        // `int32_t reset`, and the two OPTIONAL trailing arguments -- which the
        // reference does not pass -- go as `has_InitialValue = 0` and an
        // ignored 0.0.
        WindDirPlusOffset = wrap_180_c(LocalVar->WindDir + NacVaneOffset);
        WindDirPlusOffsetCosF =
            lpfilter_c(std::cos(WindDirPlusOffset * D2R), LocalVar->DT, CntrPar->F_YawErr,
                       &LocalVar->FP, LocalVar->iStatus, 0, &objInst->instLPF, 0, 0.0);
        WindDirPlusOffsetSinF =
            lpfilter_c(std::sin(WindDirPlusOffset * D2R), LocalVar->DT, CntrPar->F_YawErr,
                       &LocalVar->FP, LocalVar->iStatus, 0, &objInst->instLPF, 0, 0.0);
        NacHeadingTarget =
            wrap_180_c(std::atan2(WindDirPlusOffsetSinF, WindDirPlusOffsetCosF) * R2D);

        // ! ---- Now get into the guts of the control ----
        // ! Yaw error
        // NacHeadingError = wrap_180(NacHeadingTarget - LocalVar%NacHeading)
        NacHeadingError = wrap_180_c(NacHeadingTarget - LocalVar->NacHeading);

        // ! Check for deadband
        // IF (LocalVar%WE_Vw_F .le. CntrPar%Y_uSwitch) THEN
        //     deadband = CntrPar%Y_ErrThresh(1)
        // ELSE
        //     deadband = CntrPar%Y_ErrThresh(2)
        // ENDIF
        //
        // `Y_ErrThresh` is ALLOCATABLE and the two subscripts are literals, so
        // neither is guarded -- and neither is guarded by the reference either.
        // Both elements are read unconditionally by `CheckInputs`, so the
        // shipped program always has two; the relation is stated in
        // `harness/ranges.toml` where the corpus is constrained, not asserted
        // here as a test the reference does not have.
        if (LocalVar->WE_Vw_F <= CntrPar->Y_uSwitch) {
            deadband = CntrPar->Y_ErrThresh[1 - 1];
        } else {
            deadband = CntrPar->Y_ErrThresh[2 - 1];
        }

        // ! yawing right
        // IF (YawState == 1) THEN
        //     IF (NacHeadingError .le. 0) THEN
        //         YawRateCom = 0.0
        //         YawState = 0
        //     ELSE
        //         YawRateCom = CntrPar%Y_Rate
        //         YawState = 1
        //     ENDIF
        // ! yawing left
        // ELSEIF (YawState == -1) THEN
        //     IF (NacHeadingError .ge. 0) THEN
        //         YawRateCom = 0.0
        //         YawState = 0
        //     ELSE
        //         YawRateCom = -CntrPar%Y_Rate
        //         YawState = -1
        //     ENDIF
        // ! Initiate yaw if outside yaw error threshold
        // ELSE
        //     IF (NacHeadingError .gt. deadband) THEN
        //         YawState = 1 ! yaw right
        //     ENDIF
        //     IF (NacHeadingError .lt. -deadband) THEN
        //         YawState = -1 ! yaw left
        //     ENDIF
        //     YawRateCom = 0.0 ! if YawState is not 0, start yawing on the next time step
        // ENDIF
        //
        // The `0` in the two inner tests is an INTEGER literal compared against
        // a REAL(DbKi); the conversion is exact, so `0.0` here denotes the same
        // comparison.
        //
        // THE THIRD ARM'S TWO `IF`s ARE INDEPENDENT, NOT AN ELSE-CHAIN, AND THE
        // ORDER DECIDES THE ANSWER WHEN BOTH HOLD. Both can hold at once, for a
        // NEGATIVE `deadband`: at `deadband = -5` and an error of -3, `-3 > -5`
        // and `-3 < 5` are both true and the second write wins, leaving -1. No
        // scenario configures a negative `Y_ErrThresh` and `CheckInputs` does
        // not reject one; the shape is transcribed as the reference has it, and
        // the corpus is where whether it is reachable gets decided.
        //
        // `YawState = 1` inside the `YawState == 1` arm and `YawState = -1`
        // inside the `YawState == -1` arm are both self-assignments -- the
        // reference writes them for symmetry and they change nothing. Kept, for
        // the same reason `Tidx` is kept.
        if (YawState == 1) {
            if (NacHeadingError <= 0.0) {
                // ! stop yawing
                YawRateCom = 0.0;
                YawState = 0;
            } else {
                // ! persist
                YawRateCom = CntrPar->Y_Rate;
                YawState = 1;
            }
        } else if (YawState == -1) {
            if (NacHeadingError >= 0.0) {
                // ! stop yawing
                YawRateCom = 0.0;
                YawState = 0;
            } else {
                // ! persist
                YawRateCom = -CntrPar->Y_Rate;
                YawState = -1;
            }
        } else {
            if (NacHeadingError > deadband) {
                YawState = 1;  // yaw right
            }
            if (NacHeadingError < -deadband) {
                YawState = -1;  // yaw left
            }
            YawRateCom = 0.0;
        }

        // ! Output yaw rate command in rad/s
        // avrSWAP(48) = YawRateCom * D2R
        //
        // THE ASSIGNMENT NARROWS. `avrSWAP` is REAL(ReKi) = REAL(4) and the
        // right-hand side is REAL(DbKi) = REAL(8), so the Fortran assignment
        // rounds double to single. The cast is written out because a silent
        // implicit conversion here is the one place a reader could mistake this
        // for a copy.
        avrSWAP[48 - 1] = static_cast<float>(YawRateCom * D2R);

        // ! If using open loop yaw rate control, overwrite controlled output
        // ! Open loop yaw rate control - control input in rad/s
        // IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_YawRate > 0)) THEN
        //     IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
        //         avrSWAP(48) = interp1d(CntrPar%OL_Breakpoints, CntrPar%OL_YawRate, &
        //                                LocalVar%OL_Index, ErrVar)
        //     ENDIF
        // ENDIF
        //
        // BOTH ARGUMENTS ARE WHOLE ARRAYS, so unlike `StructuralControl`'s call
        // to the same callee there is no strided row to gather: each crosses as
        // its own pointer and its own extent, which is what `SIZE(xData)` and
        // `SIZE(yData)` denote. `interp1d`'s reference ABORTS on
        // `SIZE(xData) /= SIZE(yData)` while its translation continues
        // (evidence/interp1d/reference.size-mismatch-aborts.txt), and nothing in
        // THIS procedure enforces the equality -- `Read_OL_Input` allocates both
        // from the same file read, so they agree in the shipped program, and the
        // harness draws the two extents freely.
        //
        // THE INTERPOLANT IS `LocalVar%OL_Index`, NOT `LocalVar%Time`. It reads
        // as a typo against the guard one line above, which tests `Time`; it is
        // what the reference does, and `OL_Index` is a REAL(DbKi) field the
        // open-loop reader advances. Transcribed, not corrected (P7).
        //
        // `OL_Breakpoints(1)` is unguarded, and unguarded by the reference: the
        // arm is entered on `OL_Mode > 0` alone, which does not imply the array
        // was ever allocated.
        if ((CntrPar->OL_Mode > 0) && (CntrPar->Ind_YawRate > 0)) {
            if (LocalVar->Time >= CntrPar->OL_Breakpoints[1 - 1]) {
                avrSWAP[48 - 1] = static_cast<float>(
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_YawRate, CntrPar->n_OL_YawRate, LocalVar->OL_Index,
                               ErrVar));
            }
        }

        // ! Save for debug
        // DebugVar%YawRateCom       = YawRateCom
        // DebugVar%NacHeadingTarget = NacHeadingTarget
        // DebugVar%NacVaneOffset    = NacVaneOffset
        // DebugVar%YawState         = YawState
        // DebugVar%Yaw_Err          = NacHeadingError
        //
        // `DebugVar%YawState` is REAL(DbKi) and the local is INTEGER, so this
        // one assignment converts. Written out for the same reason as the
        // narrowing above.
        DebugVar->YawRateCom = YawRateCom;
        DebugVar->NacHeadingTarget = NacHeadingTarget;
        DebugVar->NacVaneOffset = NacVaneOffset;
        DebugVar->YawState = static_cast<double>(YawState);
        DebugVar->Yaw_Err = NacHeadingError;
    // END IF
    }
}
