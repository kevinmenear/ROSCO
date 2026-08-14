// STUB -- NOT the translation. Generated from the shipped file by
// evidence/PIIController/make_stubs.py; the ONE edit is marked below.
// VIT Translation Scaffold
// Function: PIIController
// Source: Controllers.f90
// Module: Controllers
// Fortran: FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 34368cd28f12
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-14T20:28:46Z

#include "vit_types.h"

double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, int* inst) {
    // `inst` is a 1-based Fortran index into every piP array. It is read four
    // times in the THEN arm and six times in the ELSE arm, and the Fortran
    // reads it at every subscript BEFORE incrementing the dummy at the end --
    // so it is converted ONCE here, ahead of either arm. Unit #9's rule, and
    // units #1 and #4 both paid for restating an index.
    const int i = *inst - 1;

    // The result variable. The ELSE arm assigns `PIIController` twice, reading
    // the first assignment back in the second (`saturate(PIIController, ...)`),
    // so this is a genuine local carrying a value across statements -- not the
    // write-once shape PIController has.
    double PIIController_result;

    // IF (reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if (reset != 0) {
        // piP%ITerm(inst)      = I0
        // piP%ITermLast(inst)  = I0
        // piP%ITerm2(inst)     = I0
        // piP%ITermLast2(inst) = I0
        // PIIController        = I0
        //
        // Five writes of the same value, transcribed as five because the
        // Fortran writes five. This arm clears BOTH integrator channels and
        // both of their `Last` shadows. `piP%ELast` is NOT touched -- it
        // belongs to PIDController, which shares the type -- and a translation
        // that cleared it would be wrong in a way no arithmetic comparison of
        // this unit's own outputs could show.
        piP->ITerm[i] = I0;
        piP->ITermLast[i] = I0;
        piP->ITerm2[i] = I0;
        piP->ITermLast2[i] = I0;

        PIIController_result = I0;
    } else {
        // PTerm = kp*error
        //
        // Only the FIRST error gets a proportional term. `error2` is integral
        // only; that asymmetry is the whole difference between this unit and
        // two PIControllers summed.
        const double PTerm = kp * error;

        // piP%ITerm(inst)  = piP%ITerm(inst)  + DT*ki*error
        // piP%ITerm2(inst) = piP%ITerm2(inst) + DT*ki2*error2
        //
        // `*` is left-associative in both languages, so `DT*ki*error` is
        // (DT*ki)*error on both sides -- the two roundings happen in that
        // order. Transcribed shape-for-shape and NOT reassociated to
        // DT*(ki*error), which rounds differently.
        //
        // Both accumulations happen BEFORE either clamp: the Fortran writes
        // the two raw sums first and only then clamps them, so the second
        // accumulation reads the UNCLAMPED ITerm2 of the previous call, not a
        // value this call has already clipped.
        piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;
        piP->ITerm2[i] = piP->ITerm2[i] + DT * ki2 * error2;

        // piP%ITerm(inst)  = saturate(piP%ITerm(inst),  minValue, maxValue)
        // piP%ITerm2(inst) = saturate(piP%ITerm2(inst), minValue, maxValue)
        //
        // Each integrator term is clamped IN PLACE against the SAME pair of
        // bounds as the output, and the clamped values are what the sum below
        // reads -- so these writes must land before the result is computed,
        // and the anti-windup state that survives to the next call is the
        // CLAMPED one. `saturate` is called, not inlined (X1); it is this
        // campaign's unit #24 and `saturate_c` is its C entry point.
        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);
        piP->ITerm2[i] = saturate_c(piP->ITerm2[i], minValue, maxValue);

        // PIIController = PTerm + piP%ITerm(inst) + piP%ITerm2(inst)
        //
        // `+` is left-associative in both languages: (PTerm + ITerm) + ITerm2,
        // and the intermediate rounding is in that order. Not reassociated.
        PIIController_result = PTerm + piP->ITerm[i] + piP->ITerm2[i];

        // PIIController = saturate(PIIController, minValue, maxValue)
        //
        // A THIRD, separate clamp against the same two bounds: the sum
        // P + I + I2 is clamped again. It is not redundant with the two above
        // -- PTerm can carry the sum outside [minValue, maxValue] even when
        // both integrator terms are inside it, and two clamped terms can sum
        // to twice the bound. Three sites, not one restated.
        /* PIIController_result = saturate_c(...); DELETED */

        // piP%ITermLast(inst) = piP%ITerm(inst)
        //
        // Written AFTER the result, from the CLAMPED ITerm. Note what is NOT
        // here: the Fortran does NOT write `piP%ITermLast2(inst)` in this arm,
        // although the reset arm initialises it. That asymmetry is in the
        // reference and is transcribed rather than repaired -- P7, the oracle
        // is the original source. Nothing in this unit reads either `Last`
        // field; they are state for whatever reads them later, which is why
        // the harness compares them as outputs.
        piP->ITermLast[i] = piP->ITerm[i];
    }

    // inst = inst + 1
    //
    // OUTSIDE the IF -- both arms advance the instance counter. Written
    // against `*inst` directly, because the Fortran increments the dummy and
    // the caller (`objInst%instPI`) sees it.
    *inst = *inst + 1;

    return PIIController_result;
}
