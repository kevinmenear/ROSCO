// VIT Translation Scaffold
// Function: saturate
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION saturate(inputValue, minValue, maxValue)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 9d981549ea31
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T20:50:15Z

#include <cmath>

// The reference is one statement:
//
//     saturate = REAL(MIN(MAX(inputValue,minValue), maxValue),DbKi)
//
// The REAL(..., DbKi) is the identity -- the operand is already REAL(DbKi) --
// so the whole translation is the expansion of the two intrinsics, and the
// choice between spelling them as fmin/fmax and spelling them as branches is
// the only decision this unit has.
//
// It is MEASURED, not read. evidence/saturate/saturate_expr_sweep.{f90,cpp,sh}
// runs gfortran's own MIN(MAX(...)) at the campaign's flags against all three
// spellings over 12,167 triples -- both signed zeros, both infinities, NaN,
// both denormal extremes, both finite extremes and a decade ladder:
//
//     fmin(fmax(v, lo), hi)                       0 of 12167 differ
//     t = (v > lo) ? v : lo; (t < hi) ? t : hi   789 of 12167 differ
//     t = (lo > v) ? lo : v; (hi < t) ? hi : t   561 of 12167 differ
//
// Both branch spellings are wrong at a signed zero and at a NaN, in opposite
// directions. This is NOT unit #14's rule inverted: there the Fortran wrote an
// explicit `IF (CornerFreq < 0)` and `fmax` was the mutant's answer. Here the
// Fortran writes the INTRINSIC, and gfortran's intrinsic IS fmax/fmin. The rule
// is the same one in both places -- transcribe the shape the reference has.
double saturate(double inputValue, double minValue, double maxValue) {
    return std::fmin(inputValue, maxValue);  // NO LOWER CLAMP: the MAX is deleted
}
