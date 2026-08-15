// STUB -- NOT A TRANSLATION. No rate limit at all: the ELSE arm returns the
// requested signal unchanged. The state write and the increment are kept, so
// what this measures is the rate limiter and nothing else.
// VIT Translation Scaffold
// Function: ratelimit
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION ratelimit(inputSignal, minRate, maxRate, DT, reset, rlP, inst, ResetValue)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: de534d2a6f1f
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T16:17:53Z

#include "vit_types.h"

double ratelimit(double inputSignal, double minRate, double maxRate, double DT, int32_t reset, rlparams_t* rlP, int* inst, int has_ResetValue, double ResetValue) {
    // `inst` is a 1-based Fortran subscript into `rlP%LastSignal`, read twice in
    // the ELSE arm and once in the THEN arm. Named ONCE so the 0-based
    // conversion has a single site -- unit #9's rule, restated by unit #33's
    // PIController, which has the same INOUT-counter shape. The Fortran reads
    // `inst` at every subscript BEFORE incrementing the dummy at the end, so
    // this is computed before either arm.
    //
    // NO BOUNDS GUARD. `rlP%LastSignal` is `REAL(DbKi), DIMENSION(1024)` and the
    // reference subscripts it unchecked; a `i >= 0 && i < 1024` test the
    // reference does not have would be a site no admissible input can kill --
    // unit #43's finding, where four of eleven survivors sat on one such line.
    const int i = *inst - 1;

    // The result variable. The Fortran assigns `ratelimit` in both arms of the
    // IF and never reads it back, so a single local carries it.
    double ratelimit_result;

    // ResetValue_ = inputSignal
    // IF (PRESENT(ResetValue)) ResetValue_ = ResetValue
    //
    // Two statements, transcribed as two. `ResetValue` is the signature's only
    // OPTIONAL dummy; VIT's bridge carries its presence as the separate
    // `has_ResetValue` flag, so `PRESENT(...)` is that flag and not a sentinel
    // value. Written as an unconditional assignment followed by a conditional
    // overwrite, exactly as the reference is -- collapsing the pair into a
    // ternary would be one site where the reference has two.
    //
    // NOTE THIS IS COMPUTED ON EVERY CALL AND READ ONLY IN THE THEN ARM. The
    // reference does the same; it is dead work in the ELSE arm on both sides.
    double ResetValue_ = inputSignal;
    if (has_ResetValue != 0) {
        ResetValue_ = ResetValue;
    }

    // IF (reset) THEN
    // `reset` is LOGICAL(4); the generated wrapper hands it over as
    // MERGE(1_C_INT, 0_C_INT, reset), so any non-zero value is .TRUE. and the
    // test must be `!= 0` rather than `== 1` (unit #33 measured the inverted
    // MERGE at 3112 of 3532 post-integration cases).
    if (reset != 0) {
        // rlP%LastSignal(inst) = ResetValue_
        // ratelimit            = ResetValue_
        //
        // Two writes of the same value, transcribed as two because the Fortran
        // writes two: the returned value and the state carried to the next
        // call are separate outputs and a translation that set only one would
        // be wrong in a way the return value alone cannot show.
        rlP->LastSignal[i] = ResetValue_;

        ratelimit_result = ResetValue_;
    } else {
        // rate = (inputSignal - rlP%LastSignal(inst))/DT
        //
        // The unsaturated signal rate. The subtraction is parenthesised in the
        // reference and the division follows it; `/` and `-` do not associate
        // here, so the shape is unambiguous, but the rounding order is
        // (a - b) then / DT and is transcribed in that order.
        double rate = (inputSignal - rlP->LastSignal[i]) / DT;
        (void)rate;  // STUB: the whole rate computation is discarded

        // rate = saturate(rate, minRate, maxRate)
        //
        // CALLED, not inlined (X1). `saturate` is this campaign's unit #24 and
        // `saturate_c` is its C entry point; its body is
        // `fmin(fmax(v, lo), hi)`, and inlining that here would duplicate a
        // spelling whose signed-zero and NaN behaviour cost unit #24 a
        // 12,167-triple sweep to establish.
        rate = saturate_c(rate, minRate, maxRate);  // STUB: result discarded below

        // ratelimit = rlP%LastSignal(inst) + rate*DT
        //
        // Read BEFORE the write below: the result is the previous stored
        // signal plus the clamped increment. `rate*DT` is one multiply then one
        // add, in that order, matching the reference's shape.
        ratelimit_result = inputSignal;  // STUB: NO RATE LIMIT AT ALL

        // rlP%LastSignal(inst) = ratelimit
        //
        // The state write comes AFTER the result is formed, and stores the
        // result itself -- not `inputSignal`. That is the whole rate limiter:
        // the next call's rate is measured against what was COMMANDED, not
        // against what was ASKED FOR.
        rlP->LastSignal[i] = ratelimit_result;
    }

    // inst = inst + 1
    //
    // OUTSIDE the IF -- both arms advance the instance counter. Written against
    // `*inst` directly, because the Fortran increments the dummy and the caller
    // (`objInst%instRL`) sees it.
    *inst = *inst + 1;

    return ratelimit_result;
}
