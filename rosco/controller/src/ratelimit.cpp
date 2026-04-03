#include "vit_types.h"
#include "vit_translated.h"
#include <algorithm>

double ratelimit(double inputSignal, double minRate, double maxRate, double DT, int reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue) {
    // Determine reset value: use ResetValue if present, else inputSignal
    double resetValue_ = inputSignal;
    if (has_ResetValue) {
        resetValue_ = ResetValue;
    }

    double result;

    // Fortran inst is 1-based; C array is 0-based
    int idx = *inst - 1;

    if (reset) {
        rlP->LastSignal[idx] = resetValue_;
        result = resetValue_;
    } else {
        // Compute unsaturated rate
        double rate = (inputSignal - rlP->LastSignal[idx]) / DT;
        // Saturate the rate
        rate = saturate(rate, minRate, maxRate);

        result = rlP->LastSignal[idx] + rate * DT;

        rlP->LastSignal[idx] = result;
    }

    // Increment instance (Fortran: inst = inst + 1)
    *inst = *inst + 1;

    return result;
}
