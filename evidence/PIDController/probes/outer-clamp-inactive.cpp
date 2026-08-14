// COUNTING PROBE, generated from the shipped translation by
// evidence/PIDController/make_probes.py. The only edit is the marked block:
// a sentinel written into `piP%ITerm2`, a field this unit never touches and R4
// compares, so the failing count IS the number of cases that reach the arm.
// VIT Translation Scaffold
// Function: PIDController
// Source: Controllers.f90
// Module: Controllers
// Fortran: FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 9e2a63a1564a
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-14T19:12:54Z

#include "vit_types.h"

double PIDController(double error, double kp, double ki, double kd, double tf, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, objectinstances_t* objInst, localvariables_view_t* LocalVar) {
    // EFilt = LPFilter(error, DT, tf, LocalVar%FP, LocalVar%iStatus, reset, objInst%instLPF)
    //
    // FIRST STATEMENT OF THE BODY AND UNCONDITIONAL -- the comment above it in
    // the reference says "Always filter error", and the reset arm below does
    // NOT read EFilt. So this call is a side effect in the reset arm and only a
    // side effect: it advances `objInst%instLPF` and writes six
    // `LocalVar%FP%lpf1_*` array elements at that instance whether `reset` is
    // set or not. Hoisting it into the ELSE arm would be the obvious
    // "optimisation" and would be wrong on every reset call.
    //
    // `LPFilter` is CALLED, not inlined (X1): it is this campaign's unit #11
    // and `lpfilter_c` is its C entry point. Its last two parameters are VIT's
    // spelling of the OPTIONAL `InitialValue`, which this call site does not
    // pass -- so the flag is 0 and the value is the 0.0 the flag makes
    // unreadable. Writing anything else there would be a value the reference
    // never supplies.
    //
    // `objInst%instLPF` crosses as INTENT(INOUT): LPFilter's own last statement
    // is `inst = inst + 1`, and the caller sees it. That increment is the
    // reason this unit writes TWO fields of `objInst`, not one.
    const double EFilt = lpfilter_c(error, DT, tf, &LocalVar->FP,
                                    LocalVar->iStatus, reset, &objInst->instLPF,
                                    0, 0.0);

    // `objInst%instPI` is a 1-based Fortran index into every piP array, read at
    // six subscripts across the two arms. Named ONCE here so the 0-based
    // conversion has a single site -- unit #9's rule, and units #1 and #4 both
    // paid for restating an index and getting unobservable mutants for it.
    //
    // READ AFTER THE LPFilter CALL, which is where the Fortran reads it. That
    // ordering is not decorative: `lpfilter_c` takes a pointer INTO the same
    // struct, so a reader has to know that the field it advances is `instLPF`
    // and not this one before a hoist above the call is safe. It is safe, and
    // the transcription still does not depend on it.
    const int i = objInst->instPI - 1;

    // The result variable. The Fortran assigns `PIDController` in both arms of
    // the IF and never reads it back, so a single local carries it.
    double PIDController_result;

    // IF (reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if (reset != 0) {
        // piP%ITerm(instPI)     = I0
        // piP%ITermLast(instPI) = I0
        // piP%ELast(instPI)     = 0.0_DbKi
        // PIDController         = I0
        //
        // FOUR writes, of TWO values, transcribed as four because the Fortran
        // writes four. `ELast` is cleared to a LITERAL ZERO and not to `I0` --
        // it is the previous ERROR, not a previous integrator term, so it has a
        // different initial condition even though the other two share one. A
        // translation that wrote `I0` into all three would be identical on
        // every case whose `I0` is 0.0, which is exactly what this unit's only
        // call site passes (`0.0_DbKi`, Controllers.f90:346).
        //
        // ITerm2 and ITermLast2 are NOT touched here -- they belong to
        // PIIController, which shares the type -- and a translation that
        // cleared them would be wrong in a way no arithmetic comparison of this
        // unit's own return value could show.
        piP->ITerm[i] = I0;
        piP->ITermLast[i] = I0;
        piP->ELast[i] = 0.0;

        PIDController_result = I0;
    } else {
        // PTerm = kp*error
        const double PTerm = kp * error;

        // piP%ITerm(instPI) = piP%ITerm(instPI) + DT*ki*error
        //
        // `*` is left-associative in both languages, so `DT*ki*error` is
        // (DT*ki)*error on both sides -- the two roundings are in that order.
        // Transcribed shape-for-shape and NOT reassociated to DT*(ki*error),
        // which rounds differently.
        piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;

        // piP%ITerm(instPI) = saturate(piP%ITerm(instPI), minValue, maxValue)
        //
        // The integrator term is clamped IN PLACE and the clamped value is what
        // the result line below reads -- so this write must land first, and the
        // anti-windup state that survives to the next call is the CLAMPED one.
        // `saturate` is called, not inlined (X1); it is unit #24 and
        // `saturate_c` is its C entry point.
        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);

        // DTerm = kd * (EFilt - piP%ELast(instPI)) / DT
        //
        // `*` and `/` have EQUAL precedence and associate left to right in both
        // languages, so this is ((kd * (EFilt - ELast)) / DT): a multiply, then
        // a divide. NOT (kd/DT) * (EFilt - ELast), which is the algebraically
        // equal form with a different rounding. The parentheses around the
        // difference are the reference's own and are load-bearing.
        //
        // `piP%ELast` is read HERE, before the write at the bottom of the arm,
        // so the value used is the PREVIOUS call's filtered error.
        const double DTerm = kd * (EFilt - piP->ELast[i]) / DT;

        // PIDController = saturate(PTerm + piP%ITerm(instPI) + DTerm, minValue, maxValue)
        //
        // A SECOND, separate clamp with the same two bounds. It is not
        // redundant with the one above -- PTerm and DTerm can carry the sum
        // outside [minValue, maxValue] even when ITerm is inside it -- and the
        // two calls are two sites, not one restated. `+` is left-associative,
        // so the sum is (PTerm + ITerm) + DTerm in that order.
        PIDController_result = saturate_c(PTerm + piP->ITerm[i] + DTerm,
                                          minValue, maxValue);

        // piP%ITermLast(instPI) = piP%ITerm(instPI)
        // piP%ELast(instPI)     = EFilt
        //
        // Written AFTER the result, from the CLAMPED ITerm and from the
        // filtered error computed at the top. Nothing in this unit reads
        // ITermLast; it is state for whatever reads it later, which is why the
        // harness compares it as an output rather than the return value
        // catching it. `ELast` IS read by this unit -- on the next call.
        {   // PROBE
            const double raw = PTerm + piP->ITerm[i] + DTerm;
            if (raw > minValue && raw < maxValue) piP->ITerm2[i] = 1.0;
        }
        piP->ITermLast[i] = piP->ITerm[i];
        piP->ELast[i] = EFilt;
    }

    // objInst%instPI = objInst%instPI + 1
    //
    // OUTSIDE the IF -- both arms advance the instance counter. Written against
    // the struct field rather than against the local `i`, because `i` is this
    // translation's own 0-based restatement and the counter the caller sees is
    // the 1-based field.
    objInst->instPI = objInst->instPI + 1;

    return PIDController_result;
}
