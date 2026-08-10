// VIT Translation Scaffold
// Function: ColemanTransform
// Source: Functions.f90
// Module: Functions
// Fortran: SUBROUTINE ColemanTransform(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
// Source MD5: 2c870c3258ec
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-10T21:54:27Z

#include <cmath>

void ColemanTransform(double* rootMOOP, double aziAngle, int nHarmonic, double* axTOut, double* axYOut) {
    // PI is Constants.f90's own literal, NOT M_PI. It differs from pi in the
    // 12th significant digit, and that difference reaches the output: phi2 and
    // phi3 are multiplied by nHarmonic and fed to cos/sin.
    const double PI = 3.14159265359;

    // REAL(DbKi), PARAMETER :: phi2 = 2.0/3.0*PI
    // REAL(DbKi), PARAMETER :: phi3 = 4.0/3.0*PI
    // `*` and `/` are equal precedence and left-associative in both languages,
    // so `2.0/3.0*PI` is `(2.0/3.0)*PI` on both sides. Parenthesised anyway:
    // the rounding of the intermediate is what makes them the same value.
    const double phi2 = (2.0 / 3.0) * PI;
    const double phi3 = (4.0 / 3.0) * PI;

    // INTEGER(IntKi) * REAL(DbKi) promotes the integer to REAL(DbKi) before
    // multiplying, which is what the implicit int->double conversion does here.
    const double n = static_cast<double>(nHarmonic);

    // Fortran `+` is left-associative, so the three terms accumulate in source
    // order. C++ matches. The leading 2.0/3.0 multiplies the completed sum --
    // distributing it over the terms would change the rounding.
    *axTOut = (2.0 / 3.0) * (std::cos(n * (aziAngle)) * rootMOOP[0]
                           + std::cos(n * (aziAngle + phi2)) * rootMOOP[1]
                           + std::cos(n * (aziAngle + phi3)) * rootMOOP[2]);
    *axYOut = (2.0 / 3.0) * (std::sin(n * (aziAngle)) * rootMOOP[0]
                           + std::sin(n * (aziAngle + phi2)) * rootMOOP[1]
                           + std::sin(n * (aziAngle + phi3)) * rootMOOP[2]);
}
