// CONSTANT-ARGUMENT STUB -- the real translation with `CornerFreq` REPLACED by
// the literal 0.01042, the value `CntrPar%F_FlHighPassFreq` has in all 14
// `Examples/*.IN`. Everything else is unchanged.
//
// If this PASSES, the kernel cannot constrain `CornerFreq` at all: one call
// site in one scenario sees one corner frequency, so a translation that ignores
// the argument and hardcodes the number is indistinguishable from the real one.
// Unit #3's shape (`ColemanTransformInverse`'s aziOffset), at this unit.
#include "vit_types.h"

double HPFilter(double InputSignal, double DT, double CornerFreq, filterparameters_t* FP, int iStatus, int32_t reset, int* inst, int has_InitialValue, double InitialValue) {
    (void)CornerFreq;
    const double CF = 0.01042;      // <-- the argument, ignored

    const int i = *inst - 1;
    double InitialValue_ = InputSignal;
    if (has_InitialValue) InitialValue_ = InitialValue;
    if ((iStatus == 0) || (reset != 0)) {
        FP->hpf_OutputSignalLast[i] = InitialValue_;
        FP->hpf_InputSignalLast[i] = InitialValue_;
    }
    const double K = 2.0 / DT;
    const double HPFilter_result =
          K / (CF + K) * InputSignal
        - K / (CF + K) * FP->hpf_InputSignalLast[i]
        - (CF - K) / (CF + K) * FP->hpf_OutputSignalLast[i];
    FP->hpf_InputSignalLast[i] = InputSignal;
    FP->hpf_OutputSignalLast[i] = HPFilter_result;
    *inst = *inst + 1;
    return HPFilter_result;
}
