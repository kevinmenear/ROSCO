// VIT Translation Scaffold
// Function: wrap_180
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION wrap_180(x)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: ff4256e25c9d
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-13T02:47:55Z

// wrap_180 -- fold an angle in degrees into (-180, 180].
//
// The reference, transcribed statement for statement:
//
//     IF (x .le. -180.0) THEN
//         wrap_180 = x + 360.0
//     ELSEIF (x .gt. 180.0) THEN
//         wrap_180 = x - 360.0
//     ELSE
//         wrap_180 = x
//     ENDIF
//
// Three things this shape decides, none of them free to normalise (P7):
//
//   * The two comparisons are NOT symmetric. `.le.` on the low side and `.gt.`
//     on the high side make the half-open interval (-180, 180]: at x = -180 the
//     reference wraps and returns +180, at x = +180 it does not and returns
//     +180 as well. Writing `<` / `>=`, or matching the pair up, moves exactly
//     the two endpoints -- which is the whole difference between this unit and
//     a plausible one.
//   * The reference writes explicit branches, so the translation writes explicit
//     branches. `std::fmod` or a round-and-subtract is the same function on most
//     inputs and a different one at the endpoints, at a NaN, and outside
//     [-540, 540] where the reference wraps only ONCE and leaves the rest.
//   * The comparisons are false for a NaN in both languages, so a NaN falls to
//     the ELSE and is returned unchanged. Nothing is needed to preserve that
//     beyond not reordering the tests.
//
// The literals are default-REAL in the Fortran and the reference is built with
// -fdefault-real-8, so they are REAL(8) there; 180 and 360 are exactly
// representable in binary32 and binary64 alike, so the kind cannot bite here.

double wrap_180(double x) {
    if (x <= -180.0) {
        return x + 360.0;
    } else if (x > 180.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
