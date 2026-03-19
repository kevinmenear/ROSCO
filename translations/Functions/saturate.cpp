// VIT Translation
// Function: saturate
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION saturate(inputValue, minValue, maxValue)
// Status: verified

#include <algorithm>
double saturate(double val, double minval, double maxval) {
    return std::min(std::max(val, minval), maxval);
}
