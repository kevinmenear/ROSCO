// PROBE — INPUT to a measurement, not the measurement.
//
// The shipped translation with three counters added and nothing else changed.
// It answers, over the corpus the differential harness actually generates, the
// two questions the remaining survivors turn on:
//
//   e34d5cc4  `s.size() > ErrVar->n_ErrMsg_cap` -> `>=`
//             differs on exactly one input: s.size() == cap.
//   55e42d36  `n > 0 ? (size_t)n : 0` -> `... : 1`
//             differs on exactly one input: n <= 0, where it makes the view
//             one byte long instead of empty.
//
// Both are reasoning about a quantity the corpus supplies, so the honest form
// of the claim is a COUNT over the corpus rather than an argument about it.
// Run it the way a stub is run -- swapped over translations/Functions/interp1d.cpp,
// `make -s test`, `./test` -- and read the trailer on stderr.

#include "vit_types.h"

#include <algorithm>
#include <cfloat>
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
            "\nPROBE interp1d over the harness corpus\n"
            "  calls                                     %lld\n"
            "  assign_errmsg calls                       %lld\n"
            "  ErrVar->n_ErrMsg on entry, min .. max     %d .. %d\n"
            "  ErrVar->n_ErrMsg_cap, min .. max          %d .. %d\n"
            "  largest message offered to assign_errmsg  %d\n"
            "  calls with s.size() == cap  (e34d5cc4)    %lld\n"
            "  calls with n_ErrMsg <= 0    (55e42d36)    %lld\n",
            calls, assigns, n_min, n_max, cap_min, cap_max,
            (int)size_max, size_eq_cap, n_le_zero);
    }
};
Extremes g_probe;

// CHARACTER(*), PARAMETER :: RoutineName = 'interp1d'
constexpr std::string_view RoutineName = "interp1d";

// The two message literals, as views so that neither their length nor a
// `sizeof(...) - 1` appears anywhere: a length restated is a site a mutant can
// move with nothing to compare it against (units #1, #4, #21).
constexpr std::string_view MsgSizeMismatch =
    " xData and yData are not the same size";
constexpr std::string_view MsgNotIncreasing = " xData is not strictly increasing";

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one. Same shape as
// Read_OL_Input and ReadAvrSWAP.
//
// ONE capacity test in the whole unit, deliberately. The RoutineName prefix
// used to carry a second one of its own, over a hand-written `memmove`; both
// are unreachable for the same reason (the buffer is thousands of bytes and
// every message here is under forty) and one unreachable site is cheaper to
// state than two.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    ++g_probe.assigns;
    if (s.size() > g_probe.size_max) g_probe.size_max = s.size();
    if (static_cast<int>(s.size()) == ErrVar->n_ErrMsg_cap) ++g_probe.size_eq_cap;
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: interp1d: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: interp1d: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// A NEGATIVE length is the view's NOT-ALLOCATED convention, and it collapses to
// the same empty string a zero LENGTH gives -- which is correct for both: the
// reference's `RoutineName//':'//TRIM(ErrMsg)` on a zero-length ErrMsg is
// exactly `'interp1d:'`.
// No `== npos` branch: `find_last_not_of` returns `npos`, which is `SIZE_MAX`,
// and `npos + 1` is 0 by the defined wraparound of an unsigned type -- so the
// all-blank string falls out of the same expression as every other. The branch
// it replaces was a site whose only reachable input is an all-blank message
// arriving with `aviFAIL` already negative, a CONJUNCTION of two ladders that
// do not cross (unit #16), so it was a site nothing could kill.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

double interp1d(double* xData, int n_xData, double* yData, int n_yData, double xq,
                errorvariables_view_t* ErrVar) {
    // The reference's result variable is UNDEFINED until one of the three
    // branches below assigns it, and it is left so here rather than given a
    // value the reference does not promise (P6). Every branch assigns it for
    // any ordered xq; the one path that does not is `xq` NaN, where both
    // comparisons are false and no `xq <= xData(I)` can be true either.
    double interp1d_result;

    ++g_probe.calls;
    if (ErrVar->n_ErrMsg < g_probe.n_min) g_probe.n_min = ErrVar->n_ErrMsg;
    if (ErrVar->n_ErrMsg > g_probe.n_max) g_probe.n_max = ErrVar->n_ErrMsg;
    if (ErrVar->n_ErrMsg_cap < g_probe.cap_min) g_probe.cap_min = ErrVar->n_ErrMsg_cap;
    if (ErrVar->n_ErrMsg_cap > g_probe.cap_max) g_probe.cap_max = ErrVar->n_ErrMsg_cap;
    if (ErrVar->n_ErrMsg <= 0) ++g_probe.n_le_zero;

    // The reference writes `ErrVar%aviFAIL = -1` and then a message, twice,
    // in the two error branches below. Written once here so that the sentinel
    // has ONE site rather than two -- the second branch is reachable and kills
    // it, the first is not (see the header) and would have left an unkillable
    // copy behind.
    const auto fail = [&](std::string_view msg) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, msg);
    };

    // Catch Errors
    // Are xData and yData the same size?
    if (n_xData != n_yData) {
        fail(MsgSizeMismatch);
        // WRITE(ErrVar%ErrMsg,"(A,I2,A,I2,A)") " SIZE(xData) =", SIZE(xData),
        //     ' and SIZE(yData) =', SIZE(yData),' are not the same'
        // 53 characters into the 38-character record the line above just
        // allocated. The reference dies here; see the header comment.
        std::fprintf(stderr,
                     "VIT: interp1d: SIZE(xData) = %d and SIZE(yData) = %d are "
                     "not the same; the reference aborts here (End of record) "
                     "and this translation continues\n",
                     n_xData, n_yData);
    }

    // Is xData non decreasing
    for (int I_DIFF = 1; I_DIFF <= n_xData - 1; ++I_DIFF) {
        if (xData[I_DIFF] - xData[I_DIFF - 1] <= 0) {
            fail(MsgNotIncreasing);
            break;
        }
    }

    // MINVAL / MAXVAL are reductions over the whole array, not the first and
    // last element -- the check the loop above exists to report is exactly the
    // input on which those two differ. Over a zero-size array Fortran's
    // MINVAL is +HUGE and MAXVAL is -HUGE, which is what these seeds are, and
    // seeding rather than starting from element 1 is also what gfortran does.
    double xData_min = DBL_MAX;
    double xData_max = -DBL_MAX;
    for (int I = 1; I <= n_xData; ++I) {
        const double x = xData[I - 1];
        if (x < xData_min) xData_min = x;
        if (x > xData_max) xData_max = x;
    }

    // Interpolate
    if (xq <= xData_min) {
        interp1d_result = yData[0];
    } else if (xq >= xData_max) {
        interp1d_result = yData[n_xData - 1];
    } else {
        for (int I = 1; I <= n_xData; ++I) {
            if (xq <= xData[I - 1]) {
                // yData(I-1) is yData(0) at I = 1 -- one element before the
                // array. The reference reads it; unordered xData is the only
                // way to arrive here with I = 1, which is the input the
                // strictly-increasing check above reports and does not stop.
                interp1d_result = yData[I - 2] +
                                  (yData[I - 1] - yData[I - 2]) /
                                      (xData[I - 1] - xData[I - 2]) *
                                      (xq - xData[I - 2]);
                break;
            }
        }
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }

    return interp1d_result;
}
