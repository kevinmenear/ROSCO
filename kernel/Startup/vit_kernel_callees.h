// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

double lpfilter_c(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double sigma_c(double x, double x0, double x1, double y0, double y1, errorvariables_t* ErrVar);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
