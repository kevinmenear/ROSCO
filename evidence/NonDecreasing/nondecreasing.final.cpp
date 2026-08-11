// VIT Translation Scaffold
// Function: NonDecreasing
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: FUNCTION NonDecreasing(Array)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 306c23b4c5db
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T22:56:13Z

#include <cstdint>

// Fortran:
//     NonDecreasing = .TRUE.
//     DO I_DIFF = 1, size(Array) - 1
//         IF (Array(I_DIFF + 1) - Array(I_DIFF) <= 0) THEN
//             NonDecreasing = .FALSE.
//             RETURN
//         END IF
//     END DO
//
// The LOGICAL result crosses as INTEGER(C_INT). gfortran's INTEGER->LOGICAL
// extension NORMALISES rather than bit-copying -- measured here, not read:
// 0 -> .FALSE.; 1, 2, -1 and 256 all -> .TRUE. with TRANSFER(L,0) == 1.
// Returning 1/0 is therefore exact.
//
// The loop is transcribed 1-based, the way the Fortran writes it, so that the
// boundary mutant stays observable (unit #1's lesson).
// `size(Array)` is named once, as n_Array, and not restated.

int32_t NonDecreasing(double* Array, int n_Array) {
    int32_t NonDecreasing = 1;

    for (int I_DIFF = 1; I_DIFF <= n_Array - 1; ++I_DIFF) {
        if (Array[(I_DIFF + 1) - 1] - Array[I_DIFF - 1] <= 0.0) {
            NonDecreasing = 0;
            return NonDecreasing;
        }
    }

    return NonDecreasing;
}
