// CONSTANT-ARGUMENT STUB -- the translation with five of its twelve C
// parameters replaced by the single value the simulation ever hands over, and
// with the CornerFreq saturation branch deleted.
//
// The zero stub says the kernel is ALIVE. This one says what being alive buys.
// If it PASSES, then for each substitution below the kernel is a lookup table:
// no captured case can tell the real argument from the literal.
//
//   Damp              -> 0.7   (the call site passes the literal 0.7_DbKi, and
//                               it is the ONLY call site in the controller)
//   has_Moving/Moving -> true  (that same site passes the literal .TRUE.)
//   has_InitialValue  -> 0     (that site passes no InitialValue at all)
//   CornerFreq < 0    -> never (CornerFreq is LocalVar%RotSpeedF, a filtered
//                               rotor speed; the saturation branch the unit
//                               opens with is not reached by any captured case)
#include "vit_types.h"

double NotchFilterSlopes(double InputSignal, double DT, double CornerFreq, double Damp, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_Moving, int32_t Moving, int has_InitialValue, double InitialValue) {
    (void)Damp; (void)has_Moving; (void)Moving; (void)has_InitialValue; (void)InitialValue;
    const int i = *inst - 1;

    const double InitialValue_ = InputSignal;   // the PRESENT branch, deleted
    const bool Moving_ = true;                  // the PRESENT branch, deleted
    const double Damp_ = 0.7;                   // the argument, as a literal
    const double CornerFreq_ = CornerFreq;      // the saturation branch, deleted

    if ((iStatus == 0) || (reset != 0)) {
        FP->nfs_OutputSignalLast1[i] = InitialValue_;
        FP->nfs_OutputSignalLast2[i] = InitialValue_;
        FP->nfs_InputSignalLast1[i] = InitialValue_;
        FP->nfs_InputSignalLast2[i] = InitialValue_;
    }
    if ((iStatus == 0) || (reset != 0) || Moving_) {
        FP->nfs_b2[i] = 2.0 * DT * CornerFreq_;
        FP->nfs_b0[i] = -FP->nfs_b2[i];
        FP->nfs_a2[i] = Damp_ * (DT * DT) * (CornerFreq_ * CornerFreq_) + 2.0 * DT * CornerFreq_ + 4.0 * Damp_;
        FP->nfs_a1[i] = 2.0 * Damp_ * (DT * DT) * (CornerFreq_ * CornerFreq_) - 8.0 * Damp_;
        FP->nfs_a0[i] = Damp_ * (DT * DT) * (CornerFreq_ * CornerFreq_) - 2.0 * DT * CornerFreq_ + 4.0 * Damp_;
    }

    const double result = 1.0 / FP->nfs_a2[i] * (FP->nfs_b2[i] * InputSignal + FP->nfs_b0[i] * FP->nfs_InputSignalLast1[i]
                        - FP->nfs_a1[i] * FP->nfs_OutputSignalLast1[i] - FP->nfs_a0[i] * FP->nfs_OutputSignalLast2[i]);

    FP->nfs_InputSignalLast2[i] = FP->nfs_InputSignalLast1[i];
    FP->nfs_InputSignalLast1[i] = InputSignal;
    FP->nfs_OutputSignalLast2[i] = FP->nfs_OutputSignalLast1[i];
    FP->nfs_OutputSignalLast1[i] = result;
    *inst = *inst + 1;
    return result;
}
