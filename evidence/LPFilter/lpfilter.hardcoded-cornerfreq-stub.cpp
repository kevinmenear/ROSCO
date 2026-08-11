// CONSTANT-ARGUMENT STUB -- the real translation with `CornerFreq` REPLACED by
// the literal 1.57080, the value `CntrPar%F_LPFCornerFreq` has in all 14
// `Examples/*.IN` (one distinct value, and `vit_sim.py` never patches it).
// Everything else is unchanged.
//
// If this PASSES, the kernel cannot constrain `CornerFreq` at all: one call site
// in one scenario sees one corner frequency, so a translation that ignores the
// argument and hardcodes the number is indistinguishable from the real one.
// Unit #3's shape (`ColemanTransformInverse`'s aziOffset), and unit #9's at this
// same type. Note that for THIS unit CornerFreq is read only inside the
// initialisation branch, so the stub can differ from the reference only on the
// cases where `iStatus == 0 .OR. reset`.
#include "vit_types.h"

double LPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)CornerFreq;
    const double CF = 1.57080;      // <-- the argument, ignored

    const int i = *inst - 1;
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;
    if ((iStatus == 0) || (reset != 0)) {
        FP->lpf1_OutputSignalLast[i] = InitialValue_;
        FP->lpf1_InputSignalLast[i] = InitialValue_;
        FP->lpf1_a1[i] = 2.0 + CF * DT;
        FP->lpf1_a0[i] = CF * DT - 2.0;
        FP->lpf1_b1[i] = CF * DT;
        FP->lpf1_b0[i] = CF * DT;
    }
    const double LPFilter_result =
        1.0 / FP->lpf1_a1[i] * (-(FP->lpf1_a0[i] * FP->lpf1_OutputSignalLast[i])
                                + FP->lpf1_b1[i] * InputSignal
                                + FP->lpf1_b0[i] * FP->lpf1_InputSignalLast[i]);
    FP->lpf1_InputSignalLast[i] = InputSignal;
    FP->lpf1_OutputSignalLast[i] = LPFilter_result;
    *inst = *inst + 1;
    return LPFilter_result;
}
