// VIT Translation Scaffold
// Function: SecLPFilter
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: fd89750216b4
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T14:27:10Z

#include "vit_types.h"

double SecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    // `inst` is a 1-based Fortran index into every FP array. Named ONCE, here,
    // so the 0-based conversion has a single site -- the same shape units #9,
    // #11 and #13 use in this file. The increment at the end is written against
    // `*inst` directly, because the Fortran increments the dummy and the caller
    // sees it.
    const int i = *inst - 1;

    // REAL(DbKi) :: InitialValue_
    //   InitialValue_ = InputSignal
    //   IF (PRESENT(InitialValue)) InitialValue_ = InitialValue
    // Transcribed as the Fortran writes it -- a default that is then overwritten
    // -- rather than folded into a conditional expression. `has_InitialValue` is
    // VIT's PRESENT() flag, and for THIS unit it is dead in the whole campaign:
    // all eight call sites (Filters.f90:350, 351, 371, 372, 408, 426, 427 and
    // Controllers.f90:169 in the clean source) pass eight arguments, none of them
    // InitialValue. So the branch is transcribed but nothing in the simulation
    // reaches it; only the differential harness varies the flag.
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // IF ((iStatus == 0) .OR. reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if ((iStatus == 0) || (reset != 0)) {
        // Source order. All four are set to the same value, so their relative
        // order is unobservable -- transcribed in source order anyway, because
        // nothing justifies moving them, and all four are read by the filter
        // expression below or by the history shift after it.
        FP->lpf2_OutputSignalLast1[i] = InitialValue_;
        FP->lpf2_OutputSignalLast2[i] = InitialValue_;
        FP->lpf2_InputSignalLast1[i] = InitialValue_;
        FP->lpf2_InputSignalLast2[i] = InitialValue_;

        // Coefficients. The Fortran is
        //
        //   FP%lpf2_a2(inst) = DT**2.0*CornerFreq**2.0 + 4.0 + 4.0*Damp*CornerFreq*DT
        //   FP%lpf2_a1(inst) = 2.0*DT**2.0*CornerFreq**2.0 - 8.0
        //   FP%lpf2_a0(inst) = DT**2.0*CornerFreq**2.0 + 4.0 - 4.0*Damp*CornerFreq*DT
        //   FP%lpf2_b2(inst) = DT**2.0*CornerFreq**2.0
        //   FP%lpf2_b1(inst) = 2.0*DT**2.0*CornerFreq**2.0
        //   FP%lpf2_b0(inst) = DT**2.0*CornerFreq**2.0
        //
        // Two parse facts decide every parenthesis below, and both have been
        // measured on this compiler in this file rather than assumed:
        //
        // 1. `**` binds TIGHTER than `*`, so `DT**2.0*CornerFreq**2.0` is
        //    `(DT**2.0) * (CornerFreq**2.0)` and NOT `DT**(2.0*CornerFreq)**2.0`.
        //    Written as `DT * DT * CornerFreq * CornerFreq` the C++ would
        //    accumulate `((DT*DT)*CornerFreq)*CornerFreq`, a different
        //    intermediate rounding. It is the same defect the check registry
        //    names as `exponent-grouping` and the same one that cost
        //    ComputeTau2 a catastrophic divergence in the sibling campaign.
        //
        // 2. A REAL exponent of exactly 2.0 is expanded by gfortran as a
        //    multiplication, not a libm `pow` call -- so `DT**2.0` is `DT*DT`
        //    exactly. Measured for these flags at unit #13 over 200,010 values
        //    including the overflow-to-Inf and underflow-to-zero endpoints
        //    (`evidence/NotchFilter/pow_two_probe.f90`), and this unit writes
        //    the identical `**2.0` form. A REAL exponent of 3.0 or more would
        //    NOT be safe to write this way.
        //
        // `2.0*DT**2.0*CornerFreq**2.0` therefore accumulates as
        // `(2.0 * (DT*DT)) * (CornerFreq*CornerFreq)` -- left to right, with the
        // two squarings done first. Unit #13's four surviving mutants in this
        // same file were exactly this regrouping (`2.0*(x*x)` vs `(2.0*x)*x`),
        // so the grouping is observable and not a formatting choice.
        //
        // `4.0*Damp*CornerFreq*DT` is a plain left-associative chain in both
        // languages, `((4.0*Damp)*CornerFreq)*DT`, and needs no parentheses.
        // The `+ 4.0 +` / `+ 4.0 -` chains are likewise left-associative in
        // both, so the products meet the literal in source order.
        FP->lpf2_a2[i] = (DT * DT) * (CornerFreq * CornerFreq) + 4.0 + 4.0 * Damp * CornerFreq * DT;
        FP->lpf2_a1[i] = 2.0 * (DT * DT) * (CornerFreq * CornerFreq) - 8.0;
        FP->lpf2_a0[i] = (DT * DT) * (CornerFreq * CornerFreq) + 4.0 - 4.0 * Damp * CornerFreq * DT;
        FP->lpf2_b2[i] = (DT * DT) * (CornerFreq * CornerFreq);
        FP->lpf2_b1[i] = 2.0 * (DT * DT) * (CornerFreq * CornerFreq);
        FP->lpf2_b0[i] = (DT * DT) * (CornerFreq * CornerFreq);
    }

    // SecLPFilter = 1.0/FP%lpf2_a2(inst) * (FP%lpf2_b2(inst)*InputSignal
    //                                     + FP%lpf2_b1(inst)*FP%lpf2_InputSignalLast1(inst)
    //                                     + FP%lpf2_b0(inst)*FP%lpf2_InputSignalLast2(inst)
    //                                     - FP%lpf2_a1(inst)*FP%lpf2_OutputSignalLast1(inst)
    //                                     - FP%lpf2_a0(inst)*FP%lpf2_OutputSignalLast2(inst))
    //
    // Transcribed shape-for-shape, NOT algebraically. `/` and `*` are equal
    // precedence and left-associative in both languages, so `1.0/a2 * (...)` is
    // a RECIPROCAL then a multiply -- which rounds differently from `(...)/a2`.
    // The five terms inside the parentheses accumulate left to right in both, so
    // they are written in source order with no regrouping.
    const double SecLPFilter_result =
        1.0 / FP->lpf2_a2[i] * (FP->lpf2_b2[i] * InputSignal
                                + FP->lpf2_b1[i] * FP->lpf2_InputSignalLast1[i]
                                + FP->lpf2_b0[i] * FP->lpf2_InputSignalLast2[i]
                                - FP->lpf2_a1[i] * FP->lpf2_OutputSignalLast1[i]
                                - FP->lpf2_a0[i] * FP->lpf2_OutputSignalLast2[i]);

    // Save signals for next time step. Order is load-bearing here in a way
    // LPFilter's was not: each `Last2` is written FROM its `Last1` before that
    // `Last1` is overwritten, so the four assignments are a shift and cannot be
    // reordered. All four reads in the filter expression above happen first.
    FP->lpf2_InputSignalLast2[i] = FP->lpf2_InputSignalLast1[i];
    FP->lpf2_InputSignalLast1[i] = InputSignal;
    FP->lpf2_OutputSignalLast2[i] = FP->lpf2_OutputSignalLast1[i];
    FP->lpf2_OutputSignalLast1[i] = SecLPFilter_result;

    *inst = *inst + 1;

    return SecLPFilter_result;
}
