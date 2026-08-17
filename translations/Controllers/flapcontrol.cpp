// VIT Translation Scaffold
// Function: FlapControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 1d084d061209
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-17T22:13:33Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// FOUR ARMS IN ONE CHAIN, AND THE FIRST ONE IS NOT A MODE. The reference's
// `ELSEIF` chain is headed by `IF (LocalVar%iStatus == 0)`, not by a `Flp_Mode`
// test, so the initialisation arm PRE-EMPTS all three mode arms: on a case with
// `iStatus == 0` the unit never reaches the steady, PII or cyclic arm whatever
// `Flp_Mode` holds. Writing the chain mode-first would be the obvious shape and
// it is the wrong one.
//
// WHAT THE 27 SCENARIOS CAN SEE, read from coverage/line_coverage.json against
// the clean source (`FlapControl` at Controllers.f90:639-710 at 54dd134). Five
// scenarios reach this unit at all -- 3, 4, 7, 16 and 26 -- and the other 22
// never call it:
//
//   :658  IF (Flp_Mode > 0)          15,999 / 3,999 / 23,999 / 15,999 / 15,999
//   :659  IF (iStatus == 0)          the same counts
//   :660-:666  the init writes            1 hit each, in ALL FIVE
//   :668  IF (Flp_Mode == 2)              1 hit each, in ALL FIVE
//   :669/:670  the init PII loop           4 hits, SCENARIO 4 ONLY
//   :675  ELSEIF (Flp_Mode == 1)     15,998 / 3,998 / 23,998 / 15,998 / 15,998
//   :676-:678  the steady arm         scenarios 3 and 7 only
//   :681  ELSEIF (Flp_Mode == 2)      3,998 under 4; 15,998 under 16 and 26
//   :682-:686  the PII arm            SCENARIO 4 only, 15,992 / 11,994 / 15,992
//   :690  ELSEIF (Flp_Mode == 3)     15,998 under 16 and 26
//   :692-:699  the cyclic arm        scenarios 16 and 26 only
//   :704-:706  the three avrSWAP writes    every call in all five
//
// So the gate reaches all four arms, across five scenarios: mode 1 in 3 and 7,
// mode 2 in 4, mode 3 in 16 and 26, and the `iStatus == 0` head once per run in
// each. What NO scenario reaches is the `ELSE RETURN` at :707 -- the 22
// scenarios that hold `Flp_Mode` at 0 do not call this unit at all, because
// DISCON's call site is itself behind `IF (CntrPar%Flp_Mode > 0)`. The
// differential harness calls the procedure directly and does reach it.
//
// THE REFERENCE READS AN UNINITIALISED LOCAL ON ONE PATH. `RootMyb_VelErr` is
// declared and never assigned, and :670 passes `RootMyb_VelErr(K)` to
// `PIIController` as its `error`. See the declaration below; that is the one
// place in this unit where the oracle has no answer, and where it does and does
// not matter is recorded there rather than here.

#include "vit_types.h"

namespace {

// REAL(DbKi), PARAMETER :: R2D = 57.2957795130       (Constants.f90:22)
//
// ROSCO's own twelve-digit decimal, not `180.0 / M_PI` and not a longer
// expansion: the reference multiplies by exactly this literal, and the double
// nearest to it is not the double nearest to 180/pi. Same transcription rule as
// units #47, #48 and #49 applied to `PI`.
constexpr double R2D = 57.2957795130;

// INTEGER(IntKi), PARAMETER :: NP_1 = 1              (Constants.f90:25)
//
// The first rotational harmonic. It crosses into both Coleman bridges as
// `int nHarmonic`, so it stays an integer here rather than becoming a literal
// `1` at the call sites -- the reference passes the named constant and a reader
// checking the harmonic against Constants.f90 should find the name.
constexpr int NP_1 = 1;

}  // namespace

void FlapControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                 localvariables_view_t* LocalVar, objectinstances_t* objInst) {
    // INTEGER(IntKi) :: K
    int K;

    // REAL(DbKi) :: RootMyb_Vel(3)
    //
    // DECLARED BY THE REFERENCE AND READ BY NOTHING. Not transcribed: an
    // unwritten, unread local has no observable behaviour, and declaring it
    // here would only offer mutants that cannot be killed. Named in this
    // comment so a reader diffing the declarations against Controllers.f90:653
    // finds the omission accounted for rather than missing.

    // REAL(DbKi) :: RootMyb_VelErr(3)
    //
    // *** THE REFERENCE READS THIS ARRAY WITHOUT EVER WRITING IT. ***
    //
    // `RootMyb_VelErr` is a local of `SUBROUTINE FlapControl` and no statement
    // in the procedure -- or anywhere else in ROSCO -- assigns it. Line 670
    // passes `RootMyb_VelErr(K)` to `PIIController` as its `error`. In Fortran
    // that is an undefined value: whatever the stack slot holds. It is
    // transcribed here as an equally uninitialised local, because the
    // alternative -- picking a value, 0.0 being the tempting one -- would be
    // this translation asserting an answer the program it replaces does not
    // have (P7).
    //
    // WHERE IT IS OBSERVABLE, AND WHERE IT IS NOT. The only reader is the
    // `Flp_Mode == 2` branch INSIDE the `iStatus == 0` arm, and what it reaches
    // there is `PIIController`'s `error` argument -- which that callee reads
    // only on its ELSE branch:
    //
    //     IF (reset) THEN   piP%ITerm(inst) = I0 ... PIIController = I0
    //     ELSE              PTerm = kp*error ...
    //
    // so the value matters only when `LocalVar%restart` is FALSE. In the
    // SHIPPED PROGRAM that combination cannot occur: `ReadAvrSWAP` runs at the
    // top of every DISCON call, before this unit, and its last word on the
    // subject is
    //
    //     IF (LocalVar%iStatus == 0) THEN ; LocalVar%restart = 1
    //     ELSE                            ; LocalVar%restart = 0
    //
    // (rosco/controller/src/readavrswap.cpp:161-165, unit #16, transcribed from
    // ReadSetParameters.f90). `iStatus == 0` therefore IMPLIES `restart` true,
    // the init arm always takes `PIIController`'s reset branch, and the
    // undefined value is never read. That is why all 27 gate scenarios run this
    // line -- scenario 4 does, four times -- without ever depending on it.
    //
    // The differential harness draws `iStatus` and `restart` INDEPENDENTLY, so
    // its corpus is not bound by that implication. Whether it produces cases in
    // which the undefined read is live, and what is done about them, is
    // measured in evidence/FlapControl/ rather than asserted here.
    double RootMyb_VelErr[3];

    // REAL(DbKi) :: axisTilt_1P, axisYaw_1P
    // REAL(DbKi) :: Flp_axisTilt_1P, Flp_axisYaw_1P
    double axisTilt_1P, axisYaw_1P;
    double Flp_axisTilt_1P, Flp_axisYaw_1P;

    // ! Flap control
    // IF (CntrPar%Flp_Mode > 0) THEN
    if (CntrPar->Flp_Mode > 0) {
        // IF (LocalVar%iStatus == 0) THEN
        if (LocalVar->iStatus == 0) {
            // LocalVar%RootMyb_Last(1) = 0 - LocalVar%rootMOOP(1)
            // LocalVar%RootMyb_Last(2) = 0 - LocalVar%rootMOOP(2)
            // LocalVar%RootMyb_Last(3) = 0 - LocalVar%rootMOOP(3)
            //
            // `0 - x` AND NOT `-x`, AND THE DIFFERENCE IS A SIGNED ZERO. The
            // literal `0` is a default INTEGER converted to REAL(DbKi) before
            // the subtraction, so the expression is `0.0 - x`. For every finite
            // non-zero x that equals `-x`; at `x == +0.0` it is `+0.0` while
            // `-x` is `-0.0`, and the two have different bit patterns. This
            // campaign's harness compares bit patterns, and `rootMOOP` is
            // exactly 0.0 in a great many cases. Transcribed as written.
            //
            // THE SUBSCRIPTS ARE LITERAL 1, 2, 3 AND ARE NOT BOUNDED BY NumBl.
            // `rootMOOP` and `RootMyb_Last` are `DIMENSION(3)` fixed-size
            // fields, and this arm writes all three whatever `NumBl` holds --
            // unlike the loops below, which NumBl does bound.
            LocalVar->RootMyb_Last[0] = 0.0 - LocalVar->rootMOOP[0];
            LocalVar->RootMyb_Last[1] = 0.0 - LocalVar->rootMOOP[1];
            LocalVar->RootMyb_Last[2] = 0.0 - LocalVar->rootMOOP[2];

            // ! Initial Flap angle
            // LocalVar%Flp_Angle(1) = CntrPar%Flp_Angle
            // LocalVar%Flp_Angle(2) = CntrPar%Flp_Angle
            // LocalVar%Flp_Angle(3) = CntrPar%Flp_Angle
            //
            // TWO DIFFERENT `Flp_Angle`s, AND THEY ARE NOT THE SAME KIND OF
            // THING. `CntrPar%Flp_Angle` is a REAL(DbKi) SCALAR parameter --
            // the initial flap angle read from the input file -- and
            // `LocalVar%Flp_Angle` is the REAL(DbKi)(3) per-blade state. The
            // name collision is upstream's; the view structs keep both
            // (`controlparameters_view_t::Flp_Angle` is a `double`,
            // `localvariables_view_t::Flp_Angle` is a `double[3]`), so a
            // mistake here would not compile -- which is worth knowing, because
            // it means no test in this campaign is carrying that risk.
            LocalVar->Flp_Angle[0] = CntrPar->Flp_Angle;
            LocalVar->Flp_Angle[1] = CntrPar->Flp_Angle;
            LocalVar->Flp_Angle[2] = CntrPar->Flp_Angle;

            // ! Initialize controller
            // IF (CntrPar%Flp_Mode == 2) THEN
            if (CntrPar->Flp_Mode == 2) {
                // DO K = 1,LocalVar%NumBl
                //     LocalVar%Flp_Angle(K) = PIIController(RootMyb_VelErr(K), &
                //         0 - LocalVar%Flp_Angle(K), CntrPar%Flp_Kp, CntrPar%Flp_Ki, &
                //         0.05_DbKi, -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit, &
                //         LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, &
                //         objInst%instPI)
                // END DO
                //
                // The reference's own comment on the DO line reads "upstream
                // bug: K was uninitialized here" and it is upstream text at
                // 54dd134, not an annotation of this campaign's. It is stale:
                // `K` is the loop variable and is perfectly well defined. The
                // undefined quantity on this line is `RootMyb_VelErr`, which
                // the comment does not mention -- see its declaration above.
                //
                // `0 - LocalVar%Flp_Angle(K)` is the SECOND `error` argument
                // (`error2`), and it is the same `0 - x` shape as the three
                // writes above, for the same signed-zero reason. The element it
                // reads is element K, which this loop writes only as the result
                // of the very call that reads it -- so on every iteration the
                // value read is still `CntrPar%Flp_Angle`, put there by the
                // three writes above. `error2` is therefore constant across the
                // loop while `error` is not.
                //
                // `LocalVar%restart` is LOGICAL and crosses as `int8_t`;
                // `? 1 : 0` matches the `MERGE(1_C_INT, 0_C_INT, reset)` the
                // generated wrapper writes. `objInst%instPI` is INTENT(INOUT)
                // -- `PIIController` post-increments it -- so it is passed by
                // address and the caller sees the advance. THE LOOP THEREFORE
                // ADVANCES IT NumBl TIMES, which is what bounds the counter's
                // admissible range in harness/ranges.toml.
                for (K = 1; K <= LocalVar->NumBl; ++K) {
                    LocalVar->Flp_Angle[K - 1] = piicontroller_c(
                        RootMyb_VelErr[K - 1], 0.0 - LocalVar->Flp_Angle[K - 1],
                        CntrPar->Flp_Kp, CntrPar->Flp_Ki, 0.05,
                        -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit,
                        LocalVar->DT, 0.0, &LocalVar->piP,
                        LocalVar->restart ? 1 : 0, &objInst->instPI);
                }
            // ENDIF
            }

        // ! Steady flap angle
        // ELSEIF (CntrPar%Flp_Mode == 1) THEN
        } else if (CntrPar->Flp_Mode == 1) {
            // LocalVar%Flp_Angle(1) = LocalVar%Flp_Angle(1)
            // LocalVar%Flp_Angle(2) = LocalVar%Flp_Angle(2)
            // LocalVar%Flp_Angle(3) = LocalVar%Flp_Angle(3)
            //
            // THREE SELF-ASSIGNMENTS, TRANSCRIBED. They compute nothing; the
            // arm exists so that `Flp_Mode == 1` does NOT fall through to the
            // PII or cyclic arms, and the statements are the reference's way of
            // saying "hold the previous angle". They are kept rather than
            // replaced by an empty branch because an empty branch is a claim
            // about what the reference does, and this is what it does (P7).
            // They are also, deliberately, the one place in this unit where a
            // mutation cannot be killed -- see evidence/FlapControl/.
            LocalVar->Flp_Angle[0] = LocalVar->Flp_Angle[0];
            LocalVar->Flp_Angle[1] = LocalVar->Flp_Angle[1];
            LocalVar->Flp_Angle[2] = LocalVar->Flp_Angle[2];

        // ! PII flap control
        // ELSEIF (CntrPar%Flp_Mode == 2) THEN
        } else if (CntrPar->Flp_Mode == 2) {
            // DO K = 1,LocalVar%NumBl
            for (K = 1; K <= LocalVar->NumBl; ++K) {
                // ! Find flap angle command - includes an integral term to
                // ! encourage zero flap angle
                // LocalVar%Flp_Angle(K) = PIIController(-LocalVar%rootMOOPF(K), &
                //     0 - LocalVar%Flp_Angle(K), CntrPar%Flp_Kp, CntrPar%Flp_Ki, &
                //     0.05_DbKi, -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit, &
                //     LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, &
                //     objInst%instPI)
                //
                // THE FIRST ARGUMENT IS A NEGATION AND THE SECOND IS A
                // SUBTRACTION, AND THAT ASYMMETRY IS THE REFERENCE'S. `-x` and
                // `0 - x` agree on every value except a zero, where the first
                // flips the sign bit and the second clears it. Both spellings
                // appear on this one statement, three tokens apart, and they
                // are transcribed as two different operations because they are
                // two different operations.
                LocalVar->Flp_Angle[K - 1] = piicontroller_c(
                    -LocalVar->rootMOOPF[K - 1], 0.0 - LocalVar->Flp_Angle[K - 1],
                    CntrPar->Flp_Kp, CntrPar->Flp_Ki, 0.05,
                    -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit,
                    LocalVar->DT, 0.0, &LocalVar->piP,
                    LocalVar->restart ? 1 : 0, &objInst->instPI);

                // ! Saturation Limits
                // LocalVar%Flp_Angle(K) = saturate(LocalVar%Flp_Angle(K), &
                //     -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit) * R2D
                //
                // A SECOND SATURATION AT THE SAME TWO LIMITS `PIIController`
                // HAS ALREADY APPLIED, and it is not redundant: the callee
                // saturates on its ELSE branch only, so on a reset it returns
                // `I0` -- here 0.0 -- unclamped, and this line is what a
                // `minValue > 0` or `maxValue < 0` case is clamped by.
                //
                // `* R2D` IS OUTSIDE THE SATURATION AND APPLIES TO ITS RESULT,
                // so the clamp is in radians and the stored angle is in
                // degrees. `-CntrPar%Flp_MaxPit` is a negation of the same
                // field the third argument passes unnegated; both are
                // transcribed as the reference writes them.
                LocalVar->Flp_Angle[K - 1] =
                    saturate_c(LocalVar->Flp_Angle[K - 1],
                               -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit) * R2D;
            // END DO
            }

        // ! Cyclic flap Control
        // ELSEIF (CntrPar%Flp_Mode == 3) THEN
        } else if (CntrPar->Flp_Mode == 3) {
            // ! Pass rootMOOPs through the Coleman transform to get the tilt
            // ! and yaw moment axis
            // CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, NP_1, &
            //                       axisTilt_1P, axisYaw_1P)
            //
            // `rootMOOPF` is `REAL(DbKi) :: rootMOOPF(3)`, a fixed-size field,
            // so it is contiguous and crosses as a bare pointer -- no gather,
            // unlike unit #49's strided row. `axisTilt_1P` and `axisYaw_1P` are
            // the callee's two INTENT(OUT) scalars and are passed by address.
            //
            // THIS ARM IS NOT BOUNDED BY NumBl AT ALL. The Coleman pair reads
            // and writes all three blades unconditionally, so a rotor with
            // `NumBl == 0` still gets three flap angles here while the two
            // loops above would write none. That is the reference's shape.
            colemantransform_c(LocalVar->rootMOOPF, LocalVar->Azimuth, NP_1,
                               &axisTilt_1P, &axisYaw_1P);

            // ! Apply PI control
            // Flp_axisTilt_1P = PIController(axisTilt_1P, CntrPar%Flp_Kp, &
            //     CntrPar%Flp_Ki, -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit, &
            //     LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, &
            //     objInst%instPI)
            // Flp_axisYaw_1P = PIController(axisYaw_1P, ... same ...)
            //
            // TWO CALLS, IN THIS ORDER, AND THE ORDER IS LOAD-BEARING.
            // `PIController` post-increments `objInst%instPI` and subscripts
            // `piP`'s five DIMENSION(1024) arrays with it, so the tilt call
            // takes instance `n` and the yaw call instance `n+1`. Swapping them
            // would put each axis's integrator state in the other's slot -- a
            // difference invisible on the first case of a fresh `piP` and
            // permanent afterwards.
            Flp_axisTilt_1P = picontroller_c(
                axisTilt_1P, CntrPar->Flp_Kp, CntrPar->Flp_Ki,
                -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit, LocalVar->DT, 0.0,
                &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);
            Flp_axisYaw_1P = picontroller_c(
                axisYaw_1P, CntrPar->Flp_Kp, CntrPar->Flp_Ki,
                -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit, LocalVar->DT, 0.0,
                &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);

            // ! Pass direct and quadrature axis through the inverse Coleman
            // ! transform to get the commanded pitch angles
            // CALL ColemanTransformInverse(Flp_axisTilt_1P, Flp_axisYaw_1P, &
            //     LocalVar%Azimuth, NP_1, 0.0_DbKi, LocalVar%Flp_Angle)
            //
            // The last argument is the OUTPUT and it is `LocalVar%Flp_Angle`
            // itself -- the callee writes all three elements, so this arm
            // overwrites whatever the state held, unlike the PII arm which
            // writes only the first NumBl. `0.0_DbKi` is the `aziOffset`.
            colemantransforminverse_c(Flp_axisTilt_1P, Flp_axisYaw_1P,
                                      LocalVar->Azimuth, NP_1, 0.0,
                                      LocalVar->Flp_Angle);

        // ENDIF
        }

        // ! Send to AVRSwap
        // avrSWAP(120) = LocalVar%Flp_Angle(1)   ! Send flap pitch command (deg)
        // avrSWAP(121) = LocalVar%Flp_Angle(2)
        // avrSWAP(122) = LocalVar%Flp_Angle(3)
        //
        // OUTSIDE THE ELSEIF CHAIN AND INSIDE THE `Flp_Mode > 0` TEST, so all
        // three run on every arm -- including the `iStatus == 0` arm and
        // including a `Flp_Mode` of 4 or more, which reaches none of the four
        // arms and writes the state through unchanged.
        //
        // THREE FIXED SUBSCRIPTS, NOT A LOOP AND NOT BOUNDED BY NumBl. Unit #43
        // and unit #49 both write `avrSWAP` at a DRAWN index and both needed a
        // `harness/ranges.toml` bound for it; this unit needs none, because 120,
        // 121 and 122 are literals well inside the 3000-element buffer the
        // harness allocates.
        //
        // ALL THREE NARROW. `avrSWAP` is `REAL(ReKi) :: avrSWAP(*)` = REAL(4)
        // and `Flp_Angle` is REAL(DbKi) = REAL(8), so each Fortran assignment
        // rounds double to single. The casts are written out because a silent
        // implicit conversion is the one place a reader could mistake this for
        // a copy -- and because the gate's whole view of this unit is through
        // these three slots, at binary32 resolution (unit #49's rule).
        avrSWAP[119] = static_cast<float>(LocalVar->Flp_Angle[0]);
        avrSWAP[120] = static_cast<float>(LocalVar->Flp_Angle[1]);
        avrSWAP[121] = static_cast<float>(LocalVar->Flp_Angle[2]);

    // ELSE
    //     RETURN
    // ENDIF
    //
    // The `ELSE RETURN` is the whole of the `Flp_Mode <= 0` behaviour: nothing
    // is written, not even the three avrSWAP slots. Transcribed as the absence
    // of an else branch rather than as an explicit `return`, which at the end
    // of a void function would be the same instruction and one more line to
    // mutate.
    }
}
