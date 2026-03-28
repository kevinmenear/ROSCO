// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

double interp2d_c(double* xData, int n_xData, double* yData, int n_yData, double* zData, int n_zData_rows, int n_zData_cols, double xq, double yq, errorvariables_t* ErrVar);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
