// VIT Translation Scaffold
// Function: NotchFilterSlopes
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, Moving, InitialValue)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: af5e254dfd97
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T00:44:47Z

#include "vit_types.h"

double NotchFilterSlopes(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_Moving, int32_t Moving, int has_InitialValue, double InitialValue) {
    // `inst` is a 1-based Fortran index into every FP array. Named ONCE, here,
    // so the 0-based conversion has a single site -- units #1 and #4 both paid
    // for restating an index and getting unobservable mutants for it. The
    // increment at the end is written against `*inst` directly, because the
    // Fortran increments the dummy and the caller sees it.
    const int i = *inst - 1;

    // REAL(DbKi) :: InitialValue_
    //   InitialValue_ = InputSignal
    //   IF (PRESENT(InitialValue)) InitialValue_ = InitialValue
    // Transcribed as the Fortran writes it -- a default that is then overwritten
    // -- rather than folded into a conditional expression. This unit has exactly
    // ONE call site in the whole controller (clean Filters.f90:405, the IPC /
    // Flp_Mode==3 blade-root branch) and it passes no InitialValue, so
    // `has_InitialValue` is 0 in every simulated call and this branch is reached
    // by the differential harness alone.
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // LOGICAL :: Moving_
    //   Moving_ = .FALSE.
    //   IF (PRESENT(Moving)) Moving_ = Moving
    // The one live call site passes the literal `.TRUE.`, so in every simulated
    // call `Moving_` is true and the coefficient block below runs on EVERY call
    // rather than only at initialisation. That is the difference between this
    // unit and `NotchFilter`: there, 61 of 62 captured cases never read DT or
    // the frequency argument at all. Here `DT`, `CornerFreq` and `Damp` are read
    // by all 62.
    //
    // `Moving` arrives as VIT's `MERGE(1_C_INT, 0_C_INT, Moving)`, so any
    // non-zero value is .TRUE.
    bool Moving_ = false;
    if (has_Moving) Moving_ = (Moving != 0);

    // ! Saturate Corner Freq at 0
    // IF (CornerFreq < 0) THEN
    //     CornerFreq_ = 0
    // ELSE
    //     CornerFreq_ = CornerFreq
    // ENDIF
    // Written as the branch the Fortran writes, not as a `max`. Two inputs
    // separate the two spellings and the branch is the one that matches:
    // `-0.0 < 0` is FALSE, so the reference carries a negative zero through
    // (`fmax(-0.0, 0.0)` would return +0.0), and a NaN takes the ELSE in both
    // languages while `fmax` would return the other operand.
    double CornerFreq_;
    if (CornerFreq < 0) {
        CornerFreq_ = 0;
    } else {
        CornerFreq_ = CornerFreq;
    }

    // IF ((iStatus == 0) .OR. reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if ((iStatus == 0) || (reset != 0)) {
        // Source order. The four state slots are set to the same value, so
        // their relative order is unobservable; transcribed in source order
        // anyway, because nothing justifies moving them. All four are READ by
        // the filter expression below (nfs_InputSignalLast2 only through the
        // history shift), so these writes must precede it.
        FP->nfs_OutputSignalLast1[i] = InitialValue_;
        FP->nfs_OutputSignalLast2[i] = InitialValue_;
        FP->nfs_InputSignalLast1[i] = InitialValue_;
        FP->nfs_InputSignalLast2[i] = InitialValue_;
    }

    // IF ((iStatus == 0) .OR. reset .OR. Moving_) THEN
    // A SECOND guard, wider than the first: the coefficients are recomputed on a
    // moving corner frequency while the state history is not reset. The two
    // conditions are deliberately kept as two `if`s, because the Fortran has two
    // and they are not the same predicate.
    if ((iStatus == 0) || (reset != 0) || Moving_) {
        // FP%nfs_b2(inst) = 2.0 * DT * CornerFreq_
        // FP%nfs_b0(inst) = -FP%nfs_b2(inst)
        // FP%nfs_a2(inst) = Damp*DT**2.0*CornerFreq_**2.0 + 2.0*DT*CornerFreq_ + 4.0*Damp
        // FP%nfs_a1(inst) = 2.0*Damp*DT**2.0*CornerFreq_**2.0 - 8.0*Damp
        // FP%nfs_a0(inst) = Damp*DT**2.0*CornerFreq_**2.0 - 2*DT*CornerFreq_ + 4.0*Damp
        //
        // Four transcription facts, each measured or read rather than assumed:
        //
        // 1. `DT**2.0` and `CornerFreq_**2.0` have a REAL exponent, so they are
        //    NOT automatically `x*x`. gfortran emits a libm `pow` for a real
        //    exponent of 3.0 or more and the two answers differ by 1 ULP -- but
        //    for the literal 2.0 it expands to a multiply. MEASURED at this
        //    campaign's own flags (-O3 -fdefault-real-8 -fdefault-double-8
        //    -ffp-contract=off): `x**2.0` and `x*x` have identical bit patterns
        //    on 200,010 values including the overflow-to-Inf and
        //    underflow-to-zero endpoints.
        //    `evidence/NotchFilterSlopes/pow_two_probe.out.txt`.
        //
        // 2. `**` binds TIGHTER than `*`, so `Damp*DT**2.0*CornerFreq_**2.0` is
        //    `Damp * (DT*DT) * (CornerFreq_*CornerFreq_)` and NOT
        //    `((Damp*DT)*DT) * ...`. The two round differently -- four surviving
        //    mutants at unit #13 were exactly this regrouping, and they were
        //    killable, not equivalent. Every power below is parenthesised.
        //
        // 3. Everything at equal precedence associates LEFT, in both languages:
        //    `Damp * (DT*DT) * (CornerFreq_*CornerFreq_)` is
        //    `(Damp * (DT*DT)) * (CornerFreq_*CornerFreq_)`, and `A + B + C` is
        //    `(A + B) + C`. Written with the grouping implicit, because C++
        //    associates the same way; the brackets go only where the two
        //    languages would disagree.
        //
        // 4. `- 2*DT*CornerFreq_` in nfs_a0 is an INTEGER literal 2 against a
        //    REAL(8) DT, which Fortran promotes to exactly 2.0_8 -- the same
        //    value the `2.0` in nfs_b2 and nfs_a2 has. Written `2.0` here; the
        //    asymmetry is the reference's and it changes nothing.
        FP->nfs_b2[i] = 2.0 * DT * CornerFreq_;
        FP->nfs_b0[i] = -FP->nfs_b2[i];
        FP->nfs_a2[i] = Damp * (DT * DT) * (CornerFreq_ * CornerFreq_) + 2.0 * DT * CornerFreq_ + 4.0 * Damp;
        FP->nfs_a1[i] = 2.0 * Damp * (DT * DT) * (CornerFreq_ * CornerFreq_) - 8.0 * Damp;
        FP->nfs_a0[i] = Damp * (DT * DT) * (CornerFreq_ * CornerFreq_) - 2.0 * DT * CornerFreq_ + 4.0 * Damp;
    }

    // NotchFilterSlopes = 1.0/FP%nfs_a2(inst) * (FP%nfs_b2(inst)*InputSignal + FP%nfs_b0(inst)*FP%nfs_InputSignalLast1(inst)
    //                     - FP%nfs_a1(inst)*FP%nfs_OutputSignalLast1(inst)  - FP%nfs_a0(inst)*FP%nfs_OutputSignalLast2(inst))
    //
    // `/` and `*` have EQUAL precedence and associate left to right, so this is
    // a reciprocal and then a multiply -- `(1.0/a2) * (...)` -- which rounds
    // differently from dividing the sum by a2. Transcribe the shape, not the
    // algebra. The four products accumulate left to right for the same reason.
    //
    // There is no `nfs_b1`: the numerator of this filter is 2*DT*w*(1 - z^-2),
    // so the middle input tap is structurally absent rather than zero, and
    // `FP%nfs_InputSignalLast2` is carried only by the history shift below.
    const double result = 1.0 / FP->nfs_a2[i] * (FP->nfs_b2[i] * InputSignal + FP->nfs_b0[i] * FP->nfs_InputSignalLast1[i]
                        - FP->nfs_a1[i] * FP->nfs_OutputSignalLast1[i] - FP->nfs_a0[i] * FP->nfs_OutputSignalLast2[i]);

    // ! Save signals for next time step
    // Order is load-bearing: each Last2 slot must be written from its Last1 slot
    // BEFORE that Last1 slot is overwritten.
    FP->nfs_InputSignalLast2[i] = FP->nfs_InputSignalLast1[i];
    FP->nfs_InputSignalLast1[i] = InputSignal;
    FP->nfs_OutputSignalLast2[i] = FP->nfs_OutputSignalLast1[i];
    FP->nfs_OutputSignalLast1[i] = result;

    // inst = inst + 1 -- INTENT(INOUT), and the caller (objInst%instNotchSlopes)
    // reads it back.
    *inst = *inst + 1;

    return result;
}
