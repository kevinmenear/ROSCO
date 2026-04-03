#include "vit_types.h"
double HPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;  // Fortran 1-based → C 0-based

    // OPTIONAL handling
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // Initialization
    if (iStatus == 0 || reset) {
        FP->hpf_OutputSignalLast[idx] = InitialValue_;
        FP->hpf_InputSignalLast[idx] = InitialValue_;
    }

    double K = 2.0 / DT;

    // Filter
    double result = K / (CornerFreq + K) * InputSignal
                  - K / (CornerFreq + K) * FP->hpf_InputSignalLast[idx]
                  - (CornerFreq - K) / (CornerFreq + K) * FP->hpf_OutputSignalLast[idx];

    // Save signals for next time step
    FP->hpf_InputSignalLast[idx] = InputSignal;
    FP->hpf_OutputSignalLast[idx] = result;
    *inst = *inst + 1;

    return result;
}
