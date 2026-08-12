// PROBE — INPUT to a measurement, not the measurement.
//
// The shipped translation with counters added and nothing else changed. It
// answers, over the corpus the differential harness actually generates, the
// two questions the remaining survivors of `mutation/sigma.json` turn on:
//
//   7ad82e7d  `s.size() > ErrVar->n_ErrMsg_cap` -> `>=`
//             differs on exactly one input: s.size() == cap.
//   2524b715  `n > 0 ? (size_t)n : 0` -> `... : 1`
//             differs on exactly one input: n <= 0, where it makes the view
//             one byte long instead of empty.
//
// Both are reasoning about a quantity the corpus supplies, so the honest form
// of the claim is a COUNT over the corpus rather than an argument about it.
// Run it the way a stub is run -- swapped over translations/Functions/sigma.cpp,
// `make -s test`, `./test` -- and read the trailer on stderr.  `make -s test`,
// not `make -s`: the generated Makefile's FIRST target is `sigma.hpp`, so a
// bare `make` copies the header, relinks nothing and reports the PREVIOUS
// translation's verdict (unit #18, unit #22).

#include "vit_types.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// ---- the probe's counters, and nothing else changed ----------------------
struct Extremes {
    long long calls = 0;
    long long assigns = 0;
    int n_min = 1 << 30, n_max = -(1 << 30);
    int cap_min = 1 << 30, cap_max = -(1 << 30);
    size_t size_max = 0;
    long long size_eq_cap = 0;
    long long n_le_zero = 0;
    ~Extremes() {
        std::fprintf(stderr,
            "\nPROBE sigma over the harness corpus\n"
            "  calls                                     %lld\n"
            "  assign_errmsg calls                       %lld\n"
            "  ErrVar->n_ErrMsg on entry, min .. max     %d .. %d\n"
            "  ErrVar->n_ErrMsg_cap, min .. max          %d .. %d\n"
            "  largest message offered to assign_errmsg  %d\n"
            "  calls with s.size() == cap  (7ad82e7d)    %lld\n"
            "  calls with n_ErrMsg <= 0    (2524b715)    %lld\n",
            calls, assigns, n_min, n_max, cap_min, cap_max,
            (int)size_max, size_eq_cap, n_le_zero);
    }
};
Extremes g_probe;

}  // namespace

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
    ++g_probe.assigns;
    if (s.size() > g_probe.size_max) g_probe.size_max = s.size();
    if (static_cast<int>(s.size()) == ErrVar->n_ErrMsg_cap) ++g_probe.size_eq_cap;
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
    ++g_probe.calls;
    {
        const int n = ErrVar->n_ErrMsg, cap = ErrVar->n_ErrMsg_cap;
        if (n < g_probe.n_min) g_probe.n_min = n;
        if (n > g_probe.n_max) g_probe.n_max = n;
        if (cap < g_probe.cap_min) g_probe.cap_min = cap;
        if (cap > g_probe.cap_max) g_probe.cap_max = cap;
        if (n <= 0) ++g_probe.n_le_zero;
    }

    double sigma_result;

    // a3 = 2/(x0-x1)**3
    // a2 = -3*(x0+x1)/(x0-x1)**3
    // a1 = 6*x1*x0/(x0-x1)**3
    // a0 = (x0-3*x1)*x0**2/(x0-x1)**3
    //
    // The INTEGER literals 2, 3 and 6 are converted to REAL(DbKi) before the
    // arithmetic; the unary minus in `a2` applies to the whole quotient,
    // because in Fortran an add-op binds looser than `*` and `/`.
    const double x0_less_x1 = x0 - x1;
    const double x0_less_x1_cubed = x0_less_x1 * x0_less_x1 * x0_less_x1;

    const double a3 = 2.0 / x0_less_x1_cubed;
    const double a2 = -(3.0 * (x0 + x1) / x0_less_x1_cubed);
    const double a1 = 6.0 * x1 * x0 / x0_less_x1_cubed;
    const double a0 = (x0 - 3.0 * x1) * (x0 * x0) / x0_less_x1_cubed;

    if (x < x0) {
        sigma_result = y0;
    } else if (x > x1) {
        sigma_result = y1;
    } else {
        // sigma = (a3*x**3 + a2*x**2 + a1*x + a0)*(y1-y0) + y0
        // `**` binds tighter than `*`, so the powers are parenthesised: C++
        // would otherwise read `a3*x*x*x` as `((a3*x)*x)*x` and round the
        // intermediate somewhere else (1-3 ULP).
        sigma_result =
            (a3 * (x * x * x) + a2 * (x * x) + a1 * x + a0) * (y1 - y0) + y0;
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }

    return sigma_result;
}
