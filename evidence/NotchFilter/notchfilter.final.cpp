// VIT Translation Scaffold
// Function: NotchFilter
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, InitialValue)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 4d7afd533dd7
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T23:32:20Z

#include "vit_types.h"

double NotchFilter(double InputSignal, double DT, double omega, double betaNum, double betaDen, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
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
    // -- rather than folded into a conditional expression. `has_InitialValue` is
    // VIT's PRESENT() flag, and for THIS unit it is dead in the simulation:
    // NotchFilter has four call sites, only one of which is live
    // (Filters.f90:356, 112,000 calls across 6 of the 27 scenarios), and that
    // site passes no InitialValue. The other three -- the tower-top notch loop
    // at 380/386 and the Flp_Mode==2 blade loop at 413 -- have zero hits in all
    // 27 scenarios, and none of them passes it either. So the PRESENT branch is
    // reached by the differential harness alone.
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // K = 2.0/DT.  A local in the Fortran, named ONCE there; named once here.
    // It is the bilinear-transform gain and every coefficient below reads it.
    const double K = 2.0 / DT;

    // IF ((iStatus == 0) .OR. reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if ((iStatus == 0) || (reset != 0)) {
        // Source order. The four state slots are set to the same value, so
        // their relative order is unobservable; transcribed in source order
        // anyway, because nothing justifies moving them. All four are READ by
        // the filter expression below, so these writes must precede it.
        FP->nf_OutputSignalLast1[i] = InitialValue_;
        FP->nf_OutputSignalLast2[i] = InitialValue_;
        FP->nf_InputSignalLast1[i] = InitialValue_;
        FP->nf_InputSignalLast2[i] = InitialValue_;

        // FP%nf_b2(inst) = (K**2.0 + 2.0*omega*BetaNum*K + omega**2.0)/(K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
        // FP%nf_b1(inst) = (2.0*omega**2.0 - 2.0*K**2.0)  / (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
        // FP%nf_b0(inst) = (K**2.0 - 2.0*omega*BetaNum*K + omega**2.0) / (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
        // FP%nf_a1(inst) = (2.0*omega**2.0 - 2.0*K**2.0)  / (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
        // FP%nf_a0(inst) = (K**2.0 - 2.0*omega*BetaDen*K + omega**2.0)/ (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
        //
        // Three transcription facts, each measured or read rather than assumed:
        //
        // 1. `K**2.0` has a REAL exponent, so it is NOT automatically `K*K`.
        //    gfortran emits a libm `pow` for a real exponent of 3.0 or more and
        //    the two answers differ by 1 ULP -- but for the literal 2.0 it
        //    expands to a multiply. MEASURED at this campaign's own flags
        //    (-O3 -fdefault-real-8 -fdefault-double-8 -ffp-contract=off):
        //    `x**2.0` and `x*x` have identical bit patterns on 200,010 values,
        //    including the overflow-to-Inf and underflow-to-zero endpoints.
        //    `evidence/NotchFilter/pow_two_probe.f90`. So `K * K` is exact here
        //    and carries no libm call; anything above exponent 2.0 would need
        //    `std::pow`.
        //
        // 2. `**` binds TIGHTER than `*`, so `2.0*omega**2.0` is
        //    `2.0 * (omega*omega)` and NOT `(2.0*omega) * omega`. The two round
        //    differently. Every power below is parenthesised for that reason.
        //
        // 3. `2.0*omega*BetaNum*K` is four factors at equal precedence, so it
        //    associates LEFT: `((2.0*omega) * BetaNum) * K`. Written with the
        //    grouping implicit rather than with brackets, because C++ associates
        //    the same way -- the brackets go only where the two languages
        //    disagree.
        //
        // The denominator is written out FIVE times because the Fortran writes
        // it five times. That is not the unobservable restatement units #1 and
        // #4 deleted: each occurrence divides a DIFFERENT coefficient, and all
        // five coefficients appear in the filter expression below, so a mutation
        // at any one of them changes the returned value.
        FP->nf_b2[i] = ((K * K) + 2.0 * omega * betaNum * K + (omega * omega)) / ((K * K) + 2.0 * omega * betaDen * K + (omega * omega));
        FP->nf_b1[i] = (2.0 * (omega * omega) - 2.0 * (K * K)) / ((K * K) + 2.0 * omega * betaDen * K + (omega * omega));
        FP->nf_b0[i] = ((K * K) - 2.0 * omega * betaNum * K + (omega * omega)) / ((K * K) + 2.0 * omega * betaDen * K + (omega * omega));
        FP->nf_a1[i] = (2.0 * (omega * omega) - 2.0 * (K * K)) / ((K * K) + 2.0 * omega * betaDen * K + (omega * omega));
        FP->nf_a0[i] = ((K * K) - 2.0 * omega * betaDen * K + (omega * omega)) / ((K * K) + 2.0 * omega * betaDen * K + (omega * omega));
    }

    // NotchFilter = FP%nf_b2(inst)*InputSignal
    //             + FP%nf_b1(inst)*FP%nf_InputSignalLast1(inst)
    //             + FP%nf_b0(inst)*FP%nf_InputSignalLast2(inst)
    //             - FP%nf_a1(inst)*FP%nf_OutputSignalLast1(inst)
    //             - FP%nf_a0(inst)*FP%nf_OutputSignalLast2(inst)
    //
    // One Fortran statement, five products accumulated left to right. `+` and
    // `-` are equal precedence and left-associative in both languages, so the
    // partial sums are formed in source order on both sides and no bracket is
    // needed to hold the shape. Written on five lines for reading only.
    const double NotchFilter_result =
          FP->nf_b2[i] * InputSignal
        + FP->nf_b1[i] * FP->nf_InputSignalLast1[i]
        + FP->nf_b0[i] * FP->nf_InputSignalLast2[i]
        - FP->nf_a1[i] * FP->nf_OutputSignalLast1[i]
        - FP->nf_a0[i] * FP->nf_OutputSignalLast2[i];

    // Save signals for next time step. Order is load-bearing in both pairs: the
    // Last2 slot takes the OLD Last1 value, so it must be written first. And all
    // four reads in the filter expression above happen before any of these
    // writes, which is why the result is computed into a local first.
    FP->nf_InputSignalLast2[i] = FP->nf_InputSignalLast1[i];
    FP->nf_InputSignalLast1[i] = InputSignal;
    FP->nf_OutputSignalLast2[i] = FP->nf_OutputSignalLast1[i];
    FP->nf_OutputSignalLast1[i] = NotchFilter_result;

    *inst = *inst + 1;

    return NotchFilter_result;
}
