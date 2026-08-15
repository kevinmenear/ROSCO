// VIT Translation Scaffold
// Function: ResController
// Source: Controllers.f90
// Module: Controllers
// Fortran: FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 611850acde45
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T02:17:09Z

#include "vit_types.h"

double ResController(double error, double kp, double ki, double freq, double minValue, double maxValue, double DT, resparams_t* resP, int32_t reset, int* inst) {
    // Constants.f90:24 -- REAL(DbKi), PARAMETER :: PI = 3.14159265359
    //
    // The ROSCO literal, NOT M_PI. Truncated at the twelfth digit, so it is
    // 3.14159265359000006... where pi to 17 digits is 3.14159265358979312;
    // the two differ in the last four bits of the double. Unit #3
    // (ColemanTransform) and unit #21 (unwrap) both carry this same literal
    // for the same reason -- the oracle is the source (P7), not mathematics.
    const double PI = 3.14159265359;

    // `inst` is a 1-based Fortran index into all four resP arrays, and every
    // one of the twelve subscripts in this body reads it. Named ONCE so the
    // 0-based conversion has a single site -- unit #9's rule, and the same
    // shape unit #33 (PIController) used. The Fortran reads `inst` at every
    // subscript BEFORE incrementing the dummy at the end, so this is computed
    // once, ahead of both arms.
    const int i = *inst - 1;

    // The result variable.
    //
    // THE FORTRAN LEAVES IT UNASSIGNED ON THE `reset` PATH. `ResController` is
    // written only in the ELSE arm; when `reset` is .TRUE. the function
    // returns whatever the result slot happens to hold, and the caller at
    // Controllers.f90:815 assigns that to AWC_TiltYaw(Imode), which feeds the
    // inverse Coleman transform and every blade's pitch command. That is an
    // upstream ROSCO defect, not a translation choice -- see this unit's
    // section in STATUS.md for the measurement.
    //
    // A C++ function that read an uninitialised double here would be undefined
    // BEHAVIOUR rather than merely an undefined VALUE, so the local is defined
    // at 0.0. Nothing on the ELSE path can observe the initialiser: that arm
    // assigns the variable before every read of it. What the initialiser
    // decides is only what the reset path returns, and on that path the
    // reference has no answer to be identical to.
    double ResController_result = 0.0;

    // omega = 2*PI*freq
    //
    // `*` is left-associative in both languages: (2*PI)*freq, and 2*PI is a
    // compile-time product of two constants on both sides.
    const double omega = 2.0 * PI * freq;

    // !! Tustin RC
    // b0 = 4+omega**2*DT**2
    // b1 = -8+2*omega**2*DT**2
    // b2 = 4+omega**2*DT**2
    //
    // `**` binds TIGHTER than `*` in Fortran, so `omega**2*DT**2` is
    // (omega**2)*(DT**2) -- two squares, then one product -- and NOT
    // ((omega*omega)*DT)*DT, which rounds the intermediate differently.
    // gfortran expands `x**2` for an INTEGER exponent as x*x, so the squares
    // are multiplications and not `pow`.
    //
    // b1's coefficient sits INSIDE the same chain: `2*omega**2*DT**2` parses
    // as ((2*(omega**2))*(DT**2)), so the 2 multiplies the FIRST square, not
    // the finished product. `2.0 * ((omega*omega) * (DT*DT))` is a different
    // expression and a different rounding.
    //
    // b2 is written as its own statement even though it is textually b0's
    // expression: the Fortran computes it twice, and one name restated is a
    // site a mutant cannot be seen at (unit #9). Two statements, two sites.
    const double b0 = 4.0 + (omega * omega) * (DT * DT);
    const double b1 = -8.0 + (2.0 * (omega * omega)) * (DT * DT);
    const double b2 = 4.0 + (omega * omega) * (DT * DT);

    // a0 = b0*kp + 2*DT*ki
    // a1 = b1*kp
    // a2 = b2*kp - 2*DT*ki
    //
    // `2*DT*ki` is (2*DT)*ki by left associativity, which C++ reproduces
    // unparenthesised.
    const double a0 = b0 * kp + 2.0 * DT * ki;
    const double a1 = b1 * kp;
    const double a2 = b2 * kp - 2.0 * DT * ki;

    // IF (reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if (reset != 0) {
        // Initialize persistent variables/arrays, and set initial condition
        // for integrator term
        //
        //   resP%res_OutputSignalLast1(inst) = 0
        //   resP%res_OutputSignalLast2(inst) = 0
        //   resP%res_InputSignalLast1(inst)  = 0
        //   resP%res_InputSignalLast2(inst)  = 0
        //
        // Four separate writes, transcribed as four because the Fortran writes
        // four. The INTEGER literal 0 is converted to REAL(DbKi) on assignment.
        // Only slot `inst` of each array is cleared -- the other 1023 are left
        // alone, and a translation that cleared the whole array would be wrong
        // in a way this unit's own return value could never show.
        resP->res_OutputSignalLast1[i] = 0.0;
        resP->res_OutputSignalLast2[i] = 0.0;
        resP->res_InputSignalLast1[i] = 0.0;
        resP->res_InputSignalLast2[i] = 0.0;
    } else {
        // ResController = 1/b0*( -b1*resP%res_OutputSignalLast1(inst)
        //                        - b2*resP%res_OutputSignalLast2(inst)
        //                        + a0*error
        //                        + a1*resP%res_InputSignalLast1(inst)
        //                        + a2*resP%res_InputSignalLast2(inst))
        //
        // `1/b0*(...)` is a RECIPROCAL followed by a multiply, not a division
        // of the sum: `/` and `*` have equal precedence and associate left to
        // right, so it is (1/b0) * (...). The two round differently and the
        // check registry names this shape explicitly. `1` is an INTEGER
        // literal divided by a REAL(DbKi), so the division is real.
        //
        // Fortran's unary minus binds LOOSER than `*`, so `-b1*L1` is
        // -(b1*L1); the five terms then combine left to right, in the order
        // written. Transcribed term for term rather than reassociated.
        ResController_result = 1.0 / b0 * (-(b1 * resP->res_OutputSignalLast1[i])
                                           - b2 * resP->res_OutputSignalLast2[i]
                                           + a0 * error
                                           + a1 * resP->res_InputSignalLast1[i]
                                           + a2 * resP->res_InputSignalLast2[i]);

        // ResController = saturate(ResController, minValue, maxValue)
        //
        // `saturate` is CALLED, not inlined (X1); it is this campaign's unit
        // #24 and `saturate_c` is its C entry point. The clamped value is what
        // is both returned and stored, so this write must land before the
        // state updates below read it.
        ResController_result = saturate_c(ResController_result, minValue, maxValue);

        // ! Save signals for next time step
        //   resP%res_InputSignalLast2(inst)  = resP%res_InputSignalLast1(inst)
        //   resP%res_InputSignalLast1(inst)  = error
        //   resP%res_OutputSignalLast2(inst) = resP%res_OutputSignalLast1(inst)
        //   resP%res_OutputSignalLast1(inst) = ResController
        //
        // ORDER IS LOAD-BEARING. Each pair shifts Last1 into Last2 and then
        // overwrites Last1, so swapping the two lines of either pair loses the
        // older sample and duplicates the newer one. Last1 must be read before
        // it is written, on both pairs.
        //
        // The output history is stored from the CLAMPED result, so the
        // resonator's feedback runs on the saturated signal rather than the
        // raw one.
        resP->res_InputSignalLast2[i] = resP->res_InputSignalLast1[i];
        resP->res_InputSignalLast1[i] = error;
        resP->res_OutputSignalLast2[i] = resP->res_OutputSignalLast1[i];
        resP->res_OutputSignalLast1[i] = ResController_result;
    }

    // inst = inst + 1
    //
    // OUTSIDE the IF -- both arms advance the instance counter. Written
    // against `*inst` directly, because the Fortran increments the dummy and
    // the caller (`objInst%instRes`) sees it.
    *inst = *inst + 1;

    return ResController_result;
}
