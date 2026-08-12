// VIT Translation Scaffold
// Function: sigma
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION sigma(x, x0, x1, y0, y1, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 551e04b47f99
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T22:23:43Z
//
// STUB — NOT THE TRANSLATION. The `x > x1` branch DELETED; everything above x1 takes the cubic.
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature; the body is transcribed statement for statement.
//
// THE FOUR COEFFICIENTS ARE COMPUTED UNCONDITIONALLY, EXACTLY AS THE
// REFERENCE COMPUTES THEM, AND THAT IS NOT DEAD WORK ON TWO OF THE THREE
// BRANCHES ONLY BY ACCIDENT: `x0 == x1` divides by zero and every one of them
// becomes an infinity or a NaN. The reference does it, so this does it, and
// the two agree bit for bit on that input rather than by never reaching it.
//
// `(x0-x1)**3` IS WRITTEN ONCE. The reference spells it in all four
// coefficient statements; it is the same value in all four by construction,
// so three of those spellings are restatements — sites a mutant can move with
// nothing to compare against (units #1, #4, #21, and the 0.696 -> 1.000 that
// removing one bought at unit #10). What is NOT collapsed is anything the
// reference varies: `x0**2`, `x**2` and `x**3` stay where they are written.
//
// The expansion of `**` with an INTEGER exponent was MEASURED against gfortran
// on this toolchain rather than read out of the check registry:
// `evidence/sigma/int_pow_probe.{f90,cpp,sh,txt}` — 60,000 values through
// `(x0-x1)**3`, `x0**2` and `x**3`, compared as bits after TRANSFER/memcpy,
// 0 differ. Repeated multiplication, not `pow`.

#include "vit_types.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'sigma'
constexpr std::string_view RoutineName = "sigma";

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated — a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one. Same shape as
// interp1d (unit #23), Read_OL_Input and ReadAvrSWAP.
//
// UNLIKE interp1d's, this unit's capacity test is REACHABLE in principle: the
// only message it ever writes is six characters LONGER than the one it was
// handed, so an `ErrMsg` arriving at the buffer's capacity overflows it. Which
// inputs actually reach it is a question for the corpus, not for this comment
// — see evidence/sigma/README.md.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: sigma: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: sigma: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// exactly `'sigma:'`. No `== npos` branch — `find_last_not_of` returns `npos`,
// which is `SIZE_MAX`, and `npos + 1` is 0 by the defined wraparound of an
// unsigned type, so an all-blank message falls out of the same expression as
// every other.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

double sigma(double x, double x0, double x1, double y0, double y1,
             errorvariables_view_t* ErrVar) {
    // The reference's result variable is assigned on every one of the three
    // branches below — including when `x` is a NaN, where both comparisons are
    // false and the cubic runs — so there is no path that returns something the
    // reference does not promise (P6).
    double sigma_result;

    const double x0_less_x1 = x0 - x1;
    const double x0_less_x1_cubed = x0_less_x1 * x0_less_x1 * x0_less_x1;

    const double a3 = 2.0 / x0_less_x1_cubed;
    const double a2 = -(3.0 * (x0 + x1) / x0_less_x1_cubed);
    const double a1 = 6.0 * x1 * x0 / x0_less_x1_cubed;
    const double a0 = (x0 - 3.0 * x1) * (x0 * x0) / x0_less_x1_cubed;

    if (x < x0) {
        sigma_result = y0;
    } else {
        sigma_result =
            (a3 * (x * x * x) + a2 * (x * x) + a1 * x + a0) * (y1 - y0) + y0;
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }

    return sigma_result;
}
