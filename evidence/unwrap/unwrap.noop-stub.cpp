// STUB, NOT THE SHIPPED TRANSLATION. BOTH DO WHILE loops deleted -- the unit becomes `y = x`.
// Input to a red test; see evidence/unwrap/README.md.
// VIT Translation Scaffold
// Function: unwrap
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION unwrap(x, ErrVar) RESULT(y)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 0b1a2a46739c
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T23:08:19Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature; the body is transcribed statement for statement.
//
// `PI` IS NOT pi, AND IT IS NOT `M_PI`. `Constants.f90:24` declares
//
//     REAL(DbKi), PARAMETER :: PI = 3.14159265359
//
// which is the correctly-rounded double of that 12-digit DECIMAL LITERAL —
// 3.14159265359000006...  — and pi to 17 digits is 3.14159265358979312.
// The two differ by 2.069456e-13, which is 466 ULP at this magnitude. Writing
// `M_PI` here would be a translation that computes a DIFFERENT function from
// the reference on every input whose unwrapping crosses a threshold, and the
// difference is far too large to be mistaken for rounding. Measured rather than
// asserted: evidence/unwrap/pi_literal_probe.{f90,cpp,txt}.
//
// `2 * PI` IS WRITTEN ONCE. The reference spells it in both slice statements
// and it is the same value in both by construction, so the second spelling is a
// restatement — a site a mutant can move with nothing to compare against (units
// #1, #4, #21, #25). What is NOT collapsed is anything the reference varies:
// the SIGN of the shift, and the `-PI` / `+PI` the two guards test against.
//
// The multiplication is exact and so is the subtraction it becomes: `2.0 * PI`
// scales by a power of two, and IEEE-754 defines `a - b` as `a + (-b)` bit for
// bit, so `y - 2*PI` and `y + (-(2*PI))` are the same double. One helper
// therefore serves both slice statements without changing a single result bit,
// and the loop bound `i .. SIZE(x)` is stated once instead of twice.
//
// THE REFERENCE DOES NOT TERMINATE ON PART OF ITS DECLARED DOMAIN, and this
// translation does not terminate there either — deliberately. The only
// progress either `DO WHILE` makes is `y(i:) = y(i:) +/- 2*PI`, so once the
// tail's magnitude is large enough that the addition rounds back to the same
// double, the guard's value never changes. MEASURED, not reasoned:
// `7.2057594037927936e16` (2^56) is the smallest power of two at which
// `v + 2*PI == v`, and at `1e17` and at `1e300` the loop was still running
// after 17.9 billion iterations with `y(2)` unmoved
// (evidence/unwrap/reference.does-not-terminate.txt). An infinity does the
// same at any size. That is upstream ROSCO's — the sixth defect of the "the
// reference has no answer" family here, after units #17, #21 and #23 — and a
// translation that broke out of the loop would be a DIFFERENT function on
// inputs the reference simply never answers for. Nothing in this campaign's
// corpus reaches it: the harness's `_bounds` default is +/-1e3 for an array
// and its magnitude ladder is gated on `not q.dims`, so no rung can land in
// `x`. That is a fact about the corpus and it is recorded as one in
// evidence/unwrap/README.md rather than pinned in harness/ranges.toml, which
// would narrow a domain that is already narrow enough.

#include "vit_types.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'unwrap'
constexpr std::string_view RoutineName = "unwrap";

// Constants.f90:24 — REAL(DbKi), PARAMETER :: PI = 3.14159265359
constexpr double PI = 3.14159265359;

// `2 * PI`, the reference's own shift, named once. INTEGER 2 against a
// REAL(DbKi) PARAMETER: Fortran converts and folds, and so does this.
constexpr double TWO_PI = 2.0 * PI;

// `y(i:SIZE(x)) = y(i:SIZE(x)) + delta` — the reference's array section, whose
// lower bound is the loop variable and whose upper bound is the array's own
// size. `first` is the Fortran index, so the loop runs the 1-based section and
// subscripts one lower.
void shift_tail(double* y, int first, int n, double delta) {
    for (int k = first; k <= n; ++k) {
        y[k - 1] += delta;
    }
}

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated — a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one. Same shape as
// sigma (unit #25), interp1d (#23), Read_OL_Input and ReadAvrSWAP.
//
// Like sigma's and unlike the three older ones, this unit's capacity test is
// reachable IN PRINCIPLE rather than by construction: the message it writes is
// seven characters longer than the one it was handed, so an `ErrMsg` arriving
// within seven of the buffer's capacity overflows it. Whether any input in this
// campaign's corpus does is a question about the CORPUS and is answered in
// evidence/unwrap/README.md, not here.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: unwrap: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: unwrap: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): trailing blanks only, off the field's CURRENT length.
// `find_last_not_of` rather than a hand-written backward scan, for unit #15's
// and unit #17's reason: a loop written `while (n > 0 && s[n-1] == ' ')` offers
// a `> 0` -> `>= 0` mutant that reads the byte BEFORE the buffer, which is
// undefined behaviour rather than a wrong answer and which no value comparison
// can be relied on to catch.
//
// A NEGATIVE length is the view's NOT-ALLOCATED convention and collapses to the
// same empty string a zero length gives, which is right for both: the
// reference's `RoutineName//':'//TRIM(ErrMsg)` on a zero-length ErrMsg is
// exactly `'unwrap:'`. No `== npos` branch — `find_last_not_of` returns `npos`,
// which is `SIZE_MAX`, and `npos + 1` is 0 by the defined wraparound of an
// unsigned type, so an all-blank message falls out of the same expression as
// every other.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

void unwrap(double* x, int n_x, errorvariables_view_t* ErrVar, double* unwrap_result) {
    double* y = unwrap_result;

    // y = x ! set initial
    //
    // The whole result is written before anything reads it, so no element of
    // the output is ever the caller's uninitialised memory (P6). `n_x` may be
    // 0 or 1, and then the DO loop below has no iterations — which is the
    // reference's answer too: `y = x`.
    for (int k = 0; k < n_x; ++k) {
        y[k] = x[k];
    }

    // DO i = 2, SIZE(x)
    for (int i = 2; i <= n_x; ++i) {
        // DO while (y(i) - y(i-1) .LE. -PI)
        //     y(i:SIZE(x)) = y(i:SIZE(x)) + 2 * PI
        // END DO
        //
        // `.LE.` and not `.LT.`: at a difference of exactly -PI the reference
        // shifts. The two spellings differ on that one input and on no other.
        // STUB: the +2*PI loop deleted

        // DO while (y(i) - y(i-1) .GE. PI)
        //     y(i:SIZE(x)) = y(i:SIZE(x)) - 2 * PI
        // END DO
        //
        // A NaN difference — from a NaN element, or from `inf - inf` — makes
        // both guards false and leaves the tail alone, in the reference and
        // here, because IEEE comparisons against a NaN are false whichever way
        // they are written.
        // STUB: the -2*PI loop deleted
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
