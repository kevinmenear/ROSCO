// evidence/interp2d/errmsg_extremes_probe.cpp
//
// THE SHIPPED TRANSLATION WITH SIX COUNTERS AND NOTHING ELSE CHANGED, run over
// the same 1147 harness cases the green certifies. Copied from interp1d's
// probe of the same name (unit #23), which is where this shape comes from.
//
// It answers, for the four survivors that live in the two ErrMsg helpers,
// whether the corpus ever supplies the ONE input at which each mutant differs:
//
//   98926651  `s.size() > n_ErrMsg_cap` -> `>=`        differs iff size == cap
//   f0949202  `n > 0 ? (size_t)n : 0`   -> `n >= 0`    differs iff n < 0
//   c2c4e0b7  `n > 0 ? ...`             -> `n > 1`     differs iff n == 1
//   922581aa  `... : 0`                 -> `... : 1`   differs iff n <= 0
//
// A count of 0 is a statement about THIS corpus and is a blind spot, not an
// equivalence. The distinction is kept in mutation/interp2d.equivalences.json.
#include "vit_types.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>
#include <climits>
#include <cstdlib>

namespace {

struct ErrMsgCensus {
    long assigns = 0, trims = 0;
    int  size_min = INT_MAX, size_max = INT_MIN;
    int  cap_min = INT_MAX,  cap_max = INT_MIN;
    long size_eq_cap = 0;
    int  n_min = INT_MAX, n_max = INT_MIN;
    long n_negative = 0, n_zero = 0, n_one = 0;
    ~ErrMsgCensus() {
        std::fprintf(stderr,
            "\nERRMSG CENSUS over the harness corpus\n"
            "  assign_errmsg calls                        %ld\n"
            "    s.size(),        min .. max              %d .. %d\n"
            "    n_ErrMsg_cap,    min .. max              %d .. %d\n"
            "    calls with s.size() == cap  [98926651]   %ld\n"
            "  errmsg_trim calls                          %ld\n"
            "    n_ErrMsg on entry, min .. max            %d .. %d\n"
            "    calls with n <  0           [f0949202]   %ld\n"
            "    calls with n == 0           [922581aa]   %ld\n"
            "    calls with n == 1  [c2c4e0b7, 922581aa]  %ld\n",
            assigns, size_min, size_max, cap_min, cap_max, size_eq_cap,
            trims, n_min, n_max, n_negative, n_zero, n_one);
    }
};
ErrMsgCensus CENSUS;

// CHARACTER(*), PARAMETER :: RoutineName = 'interp2d'
constexpr std::string_view RoutineName = "interp2d";

// The two strictly-increasing message literals, as views, so that neither their
// length nor a `sizeof(...) - 1` appears anywhere: a length restated is a site a
// mutant can move with nothing to compare it against (units #1, #4, #21).
constexpr std::string_view MsgXNotIncreasing = " xData is not strictly increasing";
constexpr std::string_view MsgYNotIncreasing = " yData is not strictly increasing";

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one. Copied from
// interp1d (unit #23), which is the unit this one hands its slices to.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: interp2d: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    ++CENSUS.assigns;
    { const int sz = static_cast<int>(s.size()), cp = ErrVar->n_ErrMsg_cap;
      if (sz < CENSUS.size_min) CENSUS.size_min = sz;
      if (sz > CENSUS.size_max) CENSUS.size_max = sz;
      if (cp < CENSUS.cap_min) CENSUS.cap_min = cp;
      if (cp > CENSUS.cap_max) CENSUS.cap_max = cp;
      if (sz == cp) ++CENSUS.size_eq_cap; }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: interp2d: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// undefined behaviour rather than a wrong answer. A NEGATIVE length is the
// view's NOT-ALLOCATED convention and collapses to the same empty string a zero
// length gives, which is correct for both.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    ++CENSUS.trims;
    if (n < CENSUS.n_min) CENSUS.n_min = n;
    if (n > CENSUS.n_max) CENSUS.n_max = n;
    if (n < 0) ++CENSUS.n_negative;
    if (n == 0) ++CENSUS.n_zero;
    if (n == 1) ++CENSUS.n_one;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

double interp2d(double* xData, int n_xData, double* yData, int n_yData,
                double* zData, int n_zData_rows, int n_zData_cols, double xq,
                double yq, errorvariables_view_t* ErrVar) {
    // The reference's result variable is UNDEFINED until one of its five
    // assignment sites runs, and it is left so here rather than given a value
    // the reference does not promise (P6).
    double interp2d_result;

    // `ErrVar%aviFAIL = -1` stands four times in the reference, once per error
    // branch. Written once here so the sentinel has ONE site rather than four:
    // three of the four are reachable and kill it, and three unreachable copies
    // would be three mutants no input can move (unit #43).
    const auto fail = [&](std::string_view msg) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, msg);
    };

    // Error catching
    // Are xData and zData(:,1) the same size?
    if (n_xData != n_zData_cols) {
        ErrVar->aviFAIL = -1;
        // The reference's WRITE dies here; see the header comment. `fail` is
        // not used because there is no message to assign -- the reference
        // assigns none either, it formats one, and formatting is what it cannot
        // do.
        std::fprintf(stderr,
                     "VIT: interp2d: SIZE(xData) = %d and SIZE(zData,1) = %d are "
                     "not the same; the reference formats 59 characters into "
                     "ErrVar%%ErrMsg as it stands and this translation continues\n",
                     n_xData, n_zData_cols);
    }

    // Are yData and zData(1,:) the same size?
    if (n_yData != n_zData_rows) {
        ErrVar->aviFAIL = -1;
        std::fprintf(stderr,
                     "VIT: interp2d: SIZE(yData) = %d and SIZE(zData,2) = %d are "
                     "not the same; the reference formats 59 characters into "
                     "ErrVar%%ErrMsg as it stands and this translation continues\n",
                     n_yData, n_zData_rows);
    }

    // Is xData non decreasing
    for (int I_DIFF = 1; I_DIFF <= n_xData - 1; ++I_DIFF) {
        if (xData[I_DIFF] - xData[I_DIFF - 1] <= 0) {
            fail(MsgXNotIncreasing);
            break;
        }
    }

    // Is yData non decreasing
    for (int I_DIFF = 1; I_DIFF <= n_yData - 1; ++I_DIFF) {
        if (yData[I_DIFF] - yData[I_DIFF - 1] <= 0) {
            fail(MsgYNotIncreasing);
            break;
        }
    }

    // MINVAL / MAXVAL are reductions over the whole array, not the first and
    // last element -- the check the two loops above exist to report is exactly
    // the input on which those differ. Over a zero-size array Fortran's MINVAL
    // is +HUGE and MAXVAL is -HUGE, which is what these seeds are.
    double xData_min = DBL_MAX;
    double xData_max = -DBL_MAX;
    for (int I = 1; I <= n_xData; ++I) {
        const double x = xData[I - 1];
        if (x < xData_min) xData_min = x;
        if (x > xData_max) xData_max = x;
    }
    double yData_min = DBL_MAX;
    double yData_max = -DBL_MAX;
    for (int I = 1; I <= n_yData; ++I) {
        const double y = yData[I - 1];
        if (y < yData_min) yData_min = y;
        if (y > yData_max) yData_max = y;
    }

    // `zData(:,j)` -- a COLUMN, contiguous, `SIZE(zData,1)` long. Passed as a
    // pointer into zData itself, which is what a Fortran contiguous section
    // does; no copy is made on either side.
    const auto column = [&](int j) { return &zData[(j - 1) * n_zData_rows]; };

    // `zData(i,:)` -- a ROW of a column-major array, stride `n_zData_rows`,
    // `SIZE(zData,2)` long. Fortran builds a contiguous temporary to pass it to
    // an assumed-shape dummy; this is that temporary.
    //
    // `push_back` and `row.size()`, not a sized buffer and `n_zData_cols`,
    // for unit #43's reason: the extent is never negative and never small, so a
    // `cols > 0 ? cols : 0` guard is three mutants no input can distinguish,
    // and passing `n_zData_cols` rather than the buffer's own length as the
    // callee's SIZE is worse than equivalent -- a widened loop bound would then
    // write past the vector with nothing reading the extra element.
    const auto row = [&](int i) {
        std::vector<double> r;
        for (int c = 1; c <= n_zData_cols; ++c) {
            r.push_back(zData[(c - 1) * n_zData_rows + (i - 1)]);
        }
        return r;
    };

    // The DO variables are declared at function scope, exactly as the reference
    // declares them: their value AFTER a loop is read by the code below, and an
    // EXIT leaves them at the exiting iteration while a normal completion
    // leaves them one past the bound.
    //
    // `i`, `ii`, `j` and `jj` are left UNINITIALISED, as the reference leaves
    // them.
    //
    // THIS COMMENT PREVIOUSLY SAID THAT THE Y-DIRECTION HAS NO `ieee_is_nan`
    // GUARD WHERE THE X-DIRECTION DOES, AND THAT WAS A STATEMENT ABOUT THIS
    // FILE MISREAD AS A STATEMENT ABOUT THE REFERENCE. `Functions.f90` at the
    // clean baseline guards BOTH searches -- lines 231 and 257, `grep -c
    // ieee_is_nan` is 2 -- and the omission was here, at the `yq <= yData_min`
    // test below. On `yq` NaN with `xq` interior the reference returns
    // `interp1d(xData, zData(1,:), xq)` and this translation ran the y-loop to
    // completion and read `ii` without having written it. Fixed; recorded
    // first, with the wrong artifact quoted, in
    // evidence/interp2d/yq-nan-guard.MISTRANSLATION.md (C12).
    //
    // With both guards in place, neither `i` nor `ii` nor `j` nor `jj` is ever
    // read unwritten: a NaN query returns at the first test in its own
    // direction, and a non-NaN query strictly inside the extrema always
    // reaches an EXIT because some element exceeds it.
    //
    // SIX OF THE REFERENCE'S TEN INDEX ASSIGNMENTS ARE DEAD and are not
    // transcribed: every `jj = ...` and `ii = ...` that stands immediately
    // before one of the five RETURNs writes a subroutine local that nothing
    // reads afterwards -- neither is SAVEd, and the function returns. Their
    // liveness is a property of the reference, not a reading of it. Dropping
    // them is unit #43's rule applied before the sweep rather than after: a
    // dead store is a site no input can kill, and six of them would be six
    // survivors with nothing to say. The `j = ...` and `i = ...` of the same
    // four bound branches ARE live -- they subscript the slice being passed on
    // -- and are kept.
    int i;
    int ii;
    int j;
    int jj;

    // ---- Find corner indices surrounding desired interpolation point -----
    // x-direction
    if (xq <= xData_min || std::isnan(xq)) {
        // On lower x-bound, just need to find zData(yq)
        j = 1;
        return interp1d_c(yData, n_yData, column(j), n_zData_rows, yq, ErrVar);
    } else if (xq >= xData_max) {
        // On upper x-bound, just need to find zData(yq)
        j = n_xData;
        return interp1d_c(yData, n_yData, column(j), n_zData_rows, yq, ErrVar);
    } else {
        for (j = 1; j <= n_xData; ++j) {
            if (xq == xData[j - 1]) {
                // On axis, just need 1d interpolation
                return interp1d_c(yData, n_yData, column(j), n_zData_rows, yq, ErrVar);
            } else if (xq < xData[j - 1]) {
                jj = j;
                break;
            } else {
                continue;
            }
        }
    }
    j = j - 1;  // Move j back one
    // y-direction
    if (yq <= yData_min || std::isnan(yq)) {
        // On lower y-bound, just need to find zData(xq)
        i = 1;
        std::vector<double> zRow = row(i);
        return interp1d_c(xData, n_xData, zRow.data(),
                          static_cast<int>(zRow.size()), xq, ErrVar);
    } else if (yq >= yData_max) {
        // On upper y-bound, just need to find zData(xq)
        i = n_yData;
        std::vector<double> zRow = row(i);
        return interp1d_c(xData, n_xData, zRow.data(),
                          static_cast<int>(zRow.size()), xq, ErrVar);
    } else {
        for (i = 1; i <= n_yData; ++i) {
            if (yq == yData[i - 1]) {
                // On axis, just need 1d interpolation
                std::vector<double> zRow = row(i);
                return interp1d_c(xData, n_xData, zRow.data(),
                                  static_cast<int>(zRow.size()), xq, ErrVar);
            } else if (yq < yData[i - 1]) {
                ii = i;
                break;
            } else {
                continue;
            }
        }
    }
    i = i - 1;  // move i back one

    // ---- Do bilinear interpolation ----
    // Find values at corners
    //
    // The reference stages these in `REAL(DbKi), DIMENSION(2,2) :: fQ` and
    // reads each element exactly once. Four scalars rather than a 2x2, and the
    // result of the last line rather than the reference's `DIMENSION(1) :: fxy`
    // followed by `fxy(1)`, for unit #43's rule: a subscript whose only mutants
    // are out of bounds is a site no input can kill, and a one-element array is
    // nothing but such a site. The four corner READS below keep their
    // subscripts, and those are killable -- R5 makes array elements distinct.
    const double fQ_1_1 = zData[(j - 1) * n_zData_rows + (i - 1)];
    const double fQ_2_1 = zData[(j - 1) * n_zData_rows + (ii - 1)];
    const double fQ_1_2 = zData[(jj - 1) * n_zData_rows + (i - 1)];
    const double fQ_2_2 = zData[(jj - 1) * n_zData_rows + (ii - 1)];
    // Interpolate
    const double fxy1 = (xData[jj - 1] - xq) / (xData[jj - 1] - xData[j - 1]) * fQ_1_1 +
                        (xq - xData[j - 1]) / (xData[jj - 1] - xData[j - 1]) * fQ_1_2;
    const double fxy2 = (xData[jj - 1] - xq) / (xData[jj - 1] - xData[j - 1]) * fQ_2_1 +
                        (xq - xData[j - 1]) / (xData[jj - 1] - xData[j - 1]) * fQ_2_2;
    const double fxy = (yData[ii - 1] - yq) / (yData[ii - 1] - yData[i - 1]) * fxy1 +
                       (yq - yData[i - 1]) / (yData[ii - 1] - yData[i - 1]) * fxy2;

    interp2d_result = fxy;

    // Add RoutineName to error message
    //
    // REACHED ONLY ON THE BILINEAR PATH. The five early returns above are the
    // reference's own RETURNs, and each of them leaves this prefix unapplied --
    // so a message set by the strictly-increasing scan and then handed to
    // `interp1d` comes back reading `interp1d:...` with no `interp2d:` in front
    // of it. That is upstream's shape, transcribed.
    if (ErrVar->aviFAIL < 0) {
        // ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }

    return interp2d_result;
}
