// VIT Translation Scaffold
// Function: interp1d
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION interp1d(xData, yData, xq, ErrVar)
// Source MD5: 918bebc9edc1
// VIT: 0.1.0
// Status: verified
// Generated: 2026-03-18T04:41:55Z
// 1-D linear interpolation (table lookup)
// xData must be strictly increasing
// Clamps to boundary values when xq is outside the data range
//
// VIT interface: each assumed-shape array gets its own size parameter.
// n_xData and n_yData should be equal for valid input.
double interp1d(double* xData, int n_xData, double* yData, int n_yData, double xq) {
    int n = n_xData;
    // Clamp below
    if (xq <= xData[0]) {
        return yData[0];
    }
    // Clamp above
    if (xq >= xData[n - 1]) {
        return yData[n - 1];
    }
    // Linear interpolation
    for (int i = 1; i < n; i++) {
        if (xq <= xData[i]) {
            return yData[i - 1] + (yData[i] - yData[i - 1]) / (xData[i] - xData[i - 1]) * (xq - xData[i - 1]);
        }
    }
    // Fallback (should not reach here if xData is valid)
    return yData[n - 1];
}
