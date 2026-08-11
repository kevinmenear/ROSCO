// VIT Translation Scaffold
// Function: HPFilter
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION HPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 5d68082a18b1
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T17:43:37Z

#include "vit_types.h"

double HPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    // `inst` is a 1-based Fortran index into every FP array. Named ONCE, here,
    // so the 0-based conversion has a single site: unit #1 and unit #4 both paid
    // for restating an index/bound and getting unobservable mutants for it.
    // The increment at the end is written against `*inst` directly, because the
    // Fortran increments the dummy and the caller sees it.
    const int i = *inst - 1;

    // REAL(DbKi) :: InitialValue_
    //   InitialValue_ = InputSignal
    //   IF (PRESENT(InitialValue)) InitialValue_ = InitialValue
    // Transcribed as the Fortran writes it -- a default that is then overwritten
    // -- rather than folded into the conditional expression. `has_InitialValue`
    // is VIT's PRESENT() flag; it is 0 at every one of this unit's four call
    // sites in all 27 scenarios, so this line is outside both bit-exact layers
    // and only the differential harness varies it.
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // IF ((iStatus == 0) .OR. reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE.
    if ((iStatus == 0) || (reset != 0)) {
        // Fortran order: OutputSignalLast first, then InputSignalLast. Both are
        // set to the same value, so the order cannot be observed -- transcribed
        // in source order anyway, because nothing here justifies departing.
        FP->hpf_OutputSignalLast[i] = InitialValue_;
        FP->hpf_InputSignalLast[i] = InitialValue_;
    }

    // K = 2.0 / DT
    // With -fdefault-real-8 the literal 2.0 is REAL(8), so this is a double
    // divide on both sides.
    const double K = 2.0 / DT;

    // HPFilter = K/(CornerFreq + K)*InputSignal
    //          - K/(CornerFreq + K)*FP%hpf_InputSignalLast(inst)
    //          - (CornerFreq - K)/(CornerFreq + K)*FP%hpf_OutputSignalLast(inst)
    //
    // Transcribed shape-for-shape, NOT algebraically. `/` and `*` are equal
    // precedence and left-associative in both languages, so `K/(CornerFreq+K)*x`
    // is (K/(CornerFreq+K))*x on both sides -- a reciprocal then a multiply,
    // which rounds differently from K*x/(CornerFreq+K). The two subtractions
    // are left-associative in both, so the terms accumulate in source order.
    //
    // `CornerFreq + K` is written three times because the Fortran writes it
    // three times. It is NOT the unobservable kind of restatement unit #1 and
    // unit #4 removed: each of the three occurrences sits in a different term
    // of the sum, so a mutation at any one of them changes the result.
    const double HPFilter_result =
          K / (CornerFreq + K) * InputSignal
        - K / (CornerFreq + K) * FP->hpf_InputSignalLast[i]
        - (CornerFreq - K) / (CornerFreq + K) * FP->hpf_OutputSignalLast[i];

    // Save signals for next time step. The Fortran stores the RESULT, and it
    // stores it AFTER reading the previous value above -- so the read of
    // hpf_InputSignalLast(inst) must happen before this write.
    FP->hpf_InputSignalLast[i] = InputSignal;
    FP->hpf_OutputSignalLast[i] = HPFilter_result;

    *inst = *inst + 1;

    return HPFilter_result;
}
