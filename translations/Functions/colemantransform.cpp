// VIT Translation Scaffold
// Function: ColemanTransform
// Source: Functions.f90
// Module: Functions
// Fortran: SUBROUTINE ColemanTransform(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
// Source MD5: 2c870c3258ec
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-18T13:43:44Z

void colemantransform(double* rootMOOP, double aziAngle, int nHarmonic, double* axTOut, double* axYOut) {
    const double PI = 3.14159265359;
    const double phi2 = 2.0 / 3.0 * PI;
    const double phi3 = 4.0 / 3.0 * PI;

    *axTOut = 2.0 / 3.0 * (cos(nHarmonic * aziAngle) * rootMOOP[0]
            + cos(nHarmonic * (aziAngle + phi2)) * rootMOOP[1]
            + cos(nHarmonic * (aziAngle + phi3)) * rootMOOP[2]);
    *axYOut = 2.0 / 3.0 * (sin(nHarmonic * aziAngle) * rootMOOP[0]
            + sin(nHarmonic * (aziAngle + phi2)) * rootMOOP[1]
            + sin(nHarmonic * (aziAngle + phi3)) * rootMOOP[2]);
}
