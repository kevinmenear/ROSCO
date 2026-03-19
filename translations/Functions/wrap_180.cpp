// VIT Translation
// Function: wrap_180
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION wrap_180(x)
// Status: verified

double wrap_180(double x) {
    if (x <= -180.0) {
        return x + 360.0;
    } else if (x > 180.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
