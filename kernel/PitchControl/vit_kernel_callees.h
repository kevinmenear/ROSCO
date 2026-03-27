// VIT: Kernel callee bridge declarations
// Auto-generated — allows C++ translations to call
// original Fortran functions via BIND(C) bridges.

#ifndef VIT_KERNEL_CALLEES_H
#define VIT_KERNEL_CALLEES_H

#ifdef __cplusplus
extern "C" {
#endif

void activewakecontrol_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, debugvariables_t* DebugVar, objectinstances_t* objInst);
double floatingfeedback_c(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar);
void foreaftdamping_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst);
double interp1d_c(double* xData, int n_xData, double* yData, int n_yData, double xq, errorvariables_t* ErrVar);
void ipc_c(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
double lpfilter_c(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);
double picontroller_c(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst);
double pitchsaturation_c(localvariables_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar, errorvariables_t* ErrVar);
double ratelimit_c(double inputSignal, double minRate, double maxRate, double DT, int reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue);
double saturate_c(double inputValue, double minValue, double maxValue);
double seclpfilter_c(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue);

#ifdef __cplusplus
}
#endif

#endif // VIT_KERNEL_CALLEES_H
