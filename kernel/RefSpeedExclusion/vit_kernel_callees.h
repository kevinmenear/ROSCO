// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

double ratelimit_c(double inputSignal, double minRate, double maxRate, double DT, int reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
