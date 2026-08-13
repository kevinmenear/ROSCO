// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

void addtolist_c(CFI_cdesc_t* list, int element);
int32_t nondecreasing_c(double* Array, int n_Array);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
