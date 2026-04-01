// VIT Translation Scaffold
// Function: SecLPFilter
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue)
// Source MD5: fd89750216b4
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-19T01:44:37Z

#include "filterparameters_t.h"

double SecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;  // Fortran 1-based → C 0-based

    // OPTIONAL handling
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    // Initialization
    if (iStatus == 0 || reset) {
        FP->lpf2_OutputSignalLast1[idx] = InitialValue_;
        FP->lpf2_OutputSignalLast2[idx] = InitialValue_;
        FP->lpf2_InputSignalLast1[idx] = InitialValue_;
        FP->lpf2_InputSignalLast2[idx] = InitialValue_;

        // Coefficients
        FP->lpf2_a2[idx] = DT * DT * CornerFreq * CornerFreq + 4.0 + 4.0 * Damp * CornerFreq * DT;
        FP->lpf2_a1[idx] = 2.0 * DT * DT * CornerFreq * CornerFreq - 8.0;
        FP->lpf2_a0[idx] = DT * DT * CornerFreq * CornerFreq + 4.0 - 4.0 * Damp * CornerFreq * DT;
        FP->lpf2_b2[idx] = DT * DT * CornerFreq * CornerFreq;
        FP->lpf2_b1[idx] = 2.0 * DT * DT * CornerFreq * CornerFreq;
        FP->lpf2_b0[idx] = DT * DT * CornerFreq * CornerFreq;
    }

    // Filter
    double result = 1.0 / FP->lpf2_a2[idx] *
        (FP->lpf2_b2[idx] * InputSignal
         + FP->lpf2_b1[idx] * FP->lpf2_InputSignalLast1[idx]
         + FP->lpf2_b0[idx] * FP->lpf2_InputSignalLast2[idx]
         - FP->lpf2_a1[idx] * FP->lpf2_OutputSignalLast1[idx]
         - FP->lpf2_a0[idx] * FP->lpf2_OutputSignalLast2[idx]);

    // Save signals for next time step
    FP->lpf2_InputSignalLast2[idx] = FP->lpf2_InputSignalLast1[idx];
    FP->lpf2_InputSignalLast1[idx] = InputSignal;
    FP->lpf2_OutputSignalLast2[idx] = FP->lpf2_OutputSignalLast1[idx];
    FP->lpf2_OutputSignalLast1[idx] = result;

    *inst = *inst + 1;

    return result;
}
