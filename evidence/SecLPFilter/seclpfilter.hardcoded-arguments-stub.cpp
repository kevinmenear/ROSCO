// HARDCODED-ARGUMENT STUB -- X4, unit #18. What the KERNEL cannot constrain.
//
// Identical to the shipped translation except that four of the ten arguments
// are IGNORED and replaced by the literal the simulation happens to carry:
//   CornerFreq        -> 1.57080   (CntrPar%F_LPFCornerFreq, that value in all
//                                   14 Examples/*.IN)
//   Damp              -> 0.7       (the literal 0.7_DbKi written at this call
//                                   site, Filters.f90:426)
//   has_InitialValue  -> 0         (no call site in the tree passes the ninth
//                                   argument)
//   InitialValue      -> unread
// If this passes, a translation that reads none of those four is
// indistinguishable from the real one under the kernel.
#include "vit_types.h"

double SecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)CornerFreq; (void)Damp; (void)has_InitialValue; (void)InitialValue;
    const double CornerFreq_ = 1.57080;
    const double Damp_ = 0.7;

    const int i = *inst - 1;
    double InitialValue_ = InputSignal;

    if ((iStatus == 0) || (reset != 0)) {
        FP->lpf2_OutputSignalLast1[i] = InitialValue_;
        FP->lpf2_OutputSignalLast2[i] = InitialValue_;
        FP->lpf2_InputSignalLast1[i] = InitialValue_;
        FP->lpf2_InputSignalLast2[i] = InitialValue_;
        FP->lpf2_a2[i] = (DT * DT) * (CornerFreq_ * CornerFreq_) + 4.0 + 4.0 * Damp_ * CornerFreq_ * DT;
        FP->lpf2_a1[i] = 2.0 * (DT * DT) * (CornerFreq_ * CornerFreq_) - 8.0;
        FP->lpf2_a0[i] = (DT * DT) * (CornerFreq_ * CornerFreq_) + 4.0 - 4.0 * Damp_ * CornerFreq_ * DT;
        FP->lpf2_b2[i] = (DT * DT) * (CornerFreq_ * CornerFreq_);
        FP->lpf2_b1[i] = 2.0 * (DT * DT) * (CornerFreq_ * CornerFreq_);
        FP->lpf2_b0[i] = (DT * DT) * (CornerFreq_ * CornerFreq_);
    }

    const double SecLPFilter_result =
        1.0 / FP->lpf2_a2[i] * (FP->lpf2_b2[i] * InputSignal
                                + FP->lpf2_b1[i] * FP->lpf2_InputSignalLast1[i]
                                + FP->lpf2_b0[i] * FP->lpf2_InputSignalLast2[i]
                                - FP->lpf2_a1[i] * FP->lpf2_OutputSignalLast1[i]
                                - FP->lpf2_a0[i] * FP->lpf2_OutputSignalLast2[i]);

    FP->lpf2_InputSignalLast2[i] = FP->lpf2_InputSignalLast1[i];
    FP->lpf2_InputSignalLast1[i] = InputSignal;
    FP->lpf2_OutputSignalLast2[i] = FP->lpf2_OutputSignalLast1[i];
    FP->lpf2_OutputSignalLast1[i] = SecLPFilter_result;

    *inst = *inst + 1;
    return SecLPFilter_result;
}
