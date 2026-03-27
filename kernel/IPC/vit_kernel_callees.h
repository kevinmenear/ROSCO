// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

void colemantransform_c(double* rootMOOP, double aziAngle, int nHarmonic, double* axTOut, double* axYOut);
void colemantransforminverse_c(double axTIn, double axYIn, double aziAngle, int nHarmonic, double aziOffset, double* PitComIPC);
double lpfilter_c(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double picontroller_c(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst);
double sigma_c(double x, double x0, double x1, double y0, double y1, errorvariables_t* ErrVar);
double wrap_360_c(double x);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
