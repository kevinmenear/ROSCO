// VIT Translation Scaffold
// Function: wrap_360
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION wrap_360(x)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 4fdadda25916
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-13T03:20:02Z

// wrap_360 -- fold an angle in degrees into [0, 360).
//
// The reference, transcribed statement for statement:
//
//     IF (x .lt. 0.0) THEN
//         wrap_360 = x + 360.0
//     ELSEIF (x .ge. 360.0) THEN
//         wrap_360 = x - 360.0
//     ELSE
//         wrap_360 = x
//     ENDIF
//
// Four things this shape decides, none of them free to normalise (P7):
//
//   * The two comparisons are NOT symmetric, and they are the OPPOSITE pair from
//     `wrap_180` two screens up. There the low test is `.le.` and the high test
//     is `.gt.`, making (-180, 180]; here the low test is `.lt.` and the high
//     test is `.ge.`, making [0, 360). So at x = 0 the reference does NOT wrap
//     and returns 0, and at x = 360 it DOES and returns 0. Reading the sibling's
//     spelling across -- `<=` low, `>` high -- moves exactly those two endpoints,
//     which is the whole difference between this unit and a plausible one.
//   * The comment above the function says `0<=x<=360`, and the code says
//     `0 <= result < 360`. The code is the oracle (P7): x = 360 is wrapped to 0
//     and cannot be a result. Do not translate the comment.
//   * The reference writes explicit branches, so the translation writes explicit
//     branches. `std::fmod` or a floor-and-subtract is the same function on most
//     inputs and a different one at the endpoints, at a NaN, and outside
//     [-360, 720) where the reference wraps only ONCE and leaves the rest:
//     wrap_360(-400) is -40, not 320.
//   * A negative zero takes the ELSE arm: `-0.0 < 0.0` is false in Fortran and
//     in C++ alike, so the reference returns -0.0 unchanged rather than
//     +360.0 - and `x + 360.0` on the low arm would have given exactly 360.0,
//     a value the function is supposed never to return. Nothing is needed to
//     preserve either behaviour beyond not reordering the tests and not
//     substituting an intrinsic. The same reasoning covers a NaN: both
//     comparisons are false, so it falls to the ELSE and is returned unchanged.
//
// The literals are default-REAL in the Fortran and the reference is built with
// -fdefault-real-8, so they are REAL(8) there; 0 and 360 are exactly
// representable in binary32 and binary64 alike, so the kind cannot bite here.

double wrap_360(double x) {
    if (x < 0.0) {
        return x + 360.0;
    } else if (x >= 360.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
