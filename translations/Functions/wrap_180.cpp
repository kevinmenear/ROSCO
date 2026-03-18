// VIT Translation Scaffold
// Function: wrap_180
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION wrap_180(x)
// Source MD5: ff4256e25c9d
// VIT: 0.1.0
// Status: verified
// Generated: 2026-03-18T04:41:55Z
double wrap_180(double x) {
    if (x <= -180.0) {
        return x + 360.0;
    } else if (x > 180.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
