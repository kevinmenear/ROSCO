// VIT Translation Scaffold
// Function: PIController
// Source: Controllers.f90
// Module: Controllers
// Fortran: FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 1aeccc9d29c1
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-14T18:21:42Z

#include "vit_types.h"

double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, int* inst) {
    // `inst` is a 1-based Fortran index into every piP array, and it is read
    // four times in the ELSE arm and twice in the THEN arm. Named ONCE here so
    // the 0-based conversion has a single site -- unit #9's rule, and units #1
    // and #4 both paid for restating an index and getting unobservable mutants
    // for it. The Fortran reads `inst` at every subscript BEFORE incrementing
    // the dummy at the end, so this is computed before either arm.
    const int i = *inst - 1;

    // The result variable. The Fortran assigns `PIController` in both arms of
    // the IF and never reads it back, so a single local carries it.
    double PIController_result;

    // IF (reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if (reset != 0) {
        // piP%ITerm(inst)     = I0
        // piP%ITermLast(inst) = I0
        // PIController        = I0
        //
        // Three separate writes of the same value, transcribed as three
        // because the Fortran writes three. ITerm2, ITermLast2 and ELast are
        // NOT touched here -- they belong to PIIController and PIDController,
        // which share the type -- and a translation that cleared them would be
        // wrong in a way no arithmetic comparison of this unit's own outputs
        // could show.
        piP->ITerm[i] = I0;
        piP->ITermLast[i] = I0;

        PIController_result = I0;
    } else {
        // PTerm = kp*error
        const double PTerm = kp * error;

        // piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
        //
        // `*` is left-associative in both languages, so `DT*ki*error` is
        // (DT*ki)*error on both sides -- the two roundings are in that order.
        // Transcribed shape-for-shape and NOT reassociated to DT*(ki*error),
        // which rounds differently.
        piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;

        // piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
        //
        // The integrator term is clamped IN PLACE and the clamped value is
        // what the next line reads -- so this write must land before the
        // result is computed, and the anti-windup state that survives to the
        // next call is the CLAMPED one. `saturate` is called, not inlined
        // (X1); it is this campaign's unit #24 and `saturate_c` is its C
        // entry point.
        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);

        // PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
        //
        // A SECOND, separate clamp: the sum P + I is clamped again, with the
        // same two bounds. It is not redundant with the line above -- PTerm
        // can carry the sum outside [minValue, maxValue] even when ITerm is
        // inside it -- and the two calls are two sites, not one restated.
        PIController_result = saturate_c(PTerm + piP->ITerm[i], minValue, maxValue);

        // piP%ITermLast(inst) = piP%ITerm(inst)
        //
        // Written AFTER the result, from the CLAMPED ITerm. Nothing in this
        // unit reads ITermLast; it is state for whatever reads it later, which
        // is why the harness compares it as an output rather than the kernel
        // catching it.
        piP->ITermLast[i] = piP->ITerm[i];
    }

    // inst = inst + 1
    //
    // OUTSIDE the IF -- both arms advance the instance counter. Written
    // against `*inst` directly, because the Fortran increments the dummy and
    // the caller (`objInst%instPI`) sees it.
    *inst = *inst + 1;

    return PIController_result;
}
