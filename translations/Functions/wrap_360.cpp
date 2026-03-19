// VIT Translation
// Function: wrap_360
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION wrap_360(x)
// Status: verified

double wrap_360(double x) {
    if (x < 0.0) {
        return x + 360.0;
    } else if (x >= 360.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
