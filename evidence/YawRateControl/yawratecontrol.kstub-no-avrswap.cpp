// KERNEL RED-TEST STUB, not a translation. BOTH WRITES TO `avrSWAP(48)` DELETED.
//
// Asks: can this kernel see this unit's PRINCIPAL OUTPUT at all?
//
// `avrSWAP(48)` is the commanded yaw rate in rad/s -- the one value this
// procedure exists to produce, and the only thing the calling program reads
// from it. Everything else it writes is a debug store or an intermediate.
//
// The rest of the body is the shipped translation, unchanged: the state machine
// still runs, `YawState` still advances, the five DebugVar stores still happen
// and `LocalVar%WindDir` is still written. Only the two statements that put the
// answer into the swap array are gone. The `interp1d_c` call is kept with its
// result discarded so that the callee bridge and `LocalVar%OL_Index` are still
// reached -- unit #39's rule, so this stays a red test and does not become a
// link failure or a second perturbation.
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
        //
        // THE TWO SUBSCRIPTS ARE WRITTEN `[0]` AND `[1]`, NOT `[1 - 1]` AND
        // `[2 - 1]`, and that is unit #43's rule applied before the second sweep
        // rather than after it. The subtracted spelling keeps the reference's
        // own `(1)` and `(2)` visible, which is why it was written first; what
        // it also does is offer three mutants that no input can kill as a wrong
        // ANSWER:
        //
        //   swap_operands  '2 - 1' -> '1 - 2'   index -1, a read BEFORE the
        //                                       array -- undefined behaviour,
        //                                       not a different element
        //   arith_op       '2 - 1' -> '2 + 1'   index 3, in bounds only while
        //                                       R5 draws the extent at 4 or more
        //   const_tweak    '1 - 1' -> '1 - 2'   index -1 again
        //
        // `[0]` and `[1]` offer one const mutant each -- `0 -> 1` and `1 -> 2`,
        // both landing on a real element of an array R5 never draws shorter than
        // three -- and the corpus can distinguish both wherever the deadband is
        // observable at all. Same behaviour, same reference, fewer sites at
        // which the score cannot go red.
        if (LocalVar->WE_Vw_F <= CntrPar->Y_uSwitch) {
            deadband = CntrPar->Y_ErrThresh[0];  // Y_ErrThresh(1)
        } else {
            deadband = CntrPar->Y_ErrThresh[1];  // Y_ErrThresh(2)
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
        // KERNEL RED TEST: the write to avrSWAP(48) deleted.
        (void)avrSWAP;

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
                // KERNEL RED TEST: the open-loop write to avrSWAP(48) deleted too.
                (void)interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                                 CntrPar->OL_YawRate, CntrPar->n_OL_YawRate,
                                 LocalVar->OL_Index, ErrVar);
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
