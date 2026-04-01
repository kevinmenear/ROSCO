// VIT Translation Scaffold
// Function: NotchFilterSlopes
// Source: Filters.f90
// Module: Filters
// Fortran: FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, Moving, InitialValue)
// Source MD5: af5e254dfd97
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-19T03:19:26Z

#include "filterparameters_t.h"

double NotchFilterSlopes(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int reset, int* inst, int has_Moving, int Moving, int has_InitialValue, double InitialValue) {
    int idx = *inst - 1;  // Fortran 1-based → C 0-based

    // OPTIONAL handling
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;

    int Moving_ = 0;
    if (has_Moving) Moving_ = Moving;

    // Saturate corner frequency at 0
    double CornerFreq_ = (CornerFreq < 0.0) ? 0.0 : CornerFreq;

    // Initialize state
    if (iStatus == 0 || reset) {
        FP->nfs_OutputSignalLast1[idx] = InitialValue_;
        FP->nfs_OutputSignalLast2[idx] = InitialValue_;
        FP->nfs_InputSignalLast1[idx] = InitialValue_;
        FP->nfs_InputSignalLast2[idx] = InitialValue_;
    }

    // Compute/update coefficients (also on Moving)
    // NOTE: Parenthesization matches Fortran operator precedence where ** binds
    // tighter than *, so DT**2.0*CornerFreq_**2.0 groups as (DT**2)*(CF**2).
    if (iStatus == 0 || reset || Moving_) {
        double DT2 = DT * DT;
        double CF2 = CornerFreq_ * CornerFreq_;
        FP->nfs_b2[idx] = 2.0 * DT * CornerFreq_;
        FP->nfs_b0[idx] = -FP->nfs_b2[idx];
        FP->nfs_a2[idx] = Damp * DT2 * CF2 + 2.0 * DT * CornerFreq_ + 4.0 * Damp;
        FP->nfs_a1[idx] = 2.0 * Damp * DT2 * CF2 - 8.0 * Damp;
        FP->nfs_a0[idx] = Damp * DT2 * CF2 - 2.0 * DT * CornerFreq_ + 4.0 * Damp;
    }

    // Filter
    double result = 1.0 / FP->nfs_a2[idx] *
        (FP->nfs_b2[idx] * InputSignal
         + FP->nfs_b0[idx] * FP->nfs_InputSignalLast1[idx]
         - FP->nfs_a1[idx] * FP->nfs_OutputSignalLast1[idx]
         - FP->nfs_a0[idx] * FP->nfs_OutputSignalLast2[idx]);

    // Save signals for next time step
    FP->nfs_InputSignalLast2[idx] = FP->nfs_InputSignalLast1[idx];
    FP->nfs_InputSignalLast1[idx] = InputSignal;
    FP->nfs_OutputSignalLast2[idx] = FP->nfs_OutputSignalLast1[idx];
    FP->nfs_OutputSignalLast1[idx] = result;
    *inst = *inst + 1;

    return result;
}
