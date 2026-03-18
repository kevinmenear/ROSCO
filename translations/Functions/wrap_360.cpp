// VIT Translation Scaffold
// Function: wrap_360
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION wrap_360(x)
// Source MD5: 4fdadda25916
// VIT: 0.1.0
// Status: verified
// Generated: 2026-03-18T04:41:55Z
double wrap_360(double x) {
    if (x < 0.0) {
        return x + 360.0;
    } else if (x >= 360.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
