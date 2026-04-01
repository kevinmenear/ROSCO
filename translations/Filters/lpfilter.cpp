// VIT Translation Scaffold
// Function: LPFilter
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue)
// Source MD5: 8055ec1ca707
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-19T01:43:55Z

#include "filterparameters_t.h"

double LPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;  // Fortran 1-based → C 0-based

    // OPTIONAL handling
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // Initialization
    if (iStatus == 0 || reset) {
        FP->lpf1_OutputSignalLast[idx] = InitialValue_;
        FP->lpf1_InputSignalLast[idx] = InitialValue_;
        FP->lpf1_a1[idx] = 2.0 + CornerFreq * DT;
        FP->lpf1_a0[idx] = CornerFreq * DT - 2.0;
        FP->lpf1_b1[idx] = CornerFreq * DT;
        FP->lpf1_b0[idx] = CornerFreq * DT;
    }

    // Filter
    double result = 1.0 / FP->lpf1_a1[idx] *
        (-FP->lpf1_a0[idx] * FP->lpf1_OutputSignalLast[idx]
         + FP->lpf1_b1[idx] * InputSignal
         + FP->lpf1_b0[idx] * FP->lpf1_InputSignalLast[idx]);

    // Save signals for next time step
    FP->lpf1_InputSignalLast[idx] = InputSignal;
    FP->lpf1_OutputSignalLast[idx] = result;
    *inst = *inst + 1;

    return result;
}
