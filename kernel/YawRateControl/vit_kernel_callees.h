// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

double interp1d_c(double* xData, int n_xData, double* yData, int n_yData, double xq, errorvariables_t* ErrVar);
double lpfilter_c(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double wrap_180_c(double x);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
