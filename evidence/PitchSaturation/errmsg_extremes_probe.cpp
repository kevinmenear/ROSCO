// PROBE -- INPUT to a measurement, not the measurement.
//
// The shipped translation with one counter block added and nothing else
// changed. It answers, over the corpus the differential harness actually
// generates, the questions the five helper-function survivors turn on.
// Copied in shape from evidence/interp1d/errmsg_extremes_probe.cpp (unit #23),
// which asked two of these same questions at the same two sites.
//
//   A  `s.size() > ErrVar->n_ErrMsg_cap` -> `>=`   differs iff s.size() == cap
//   C  `n > 0 ? (size_t)n : 0` -> `n > 1 ? ...`    differs iff n == 1
//   D  `... : 0` -> `... : 1`                      differs iff n <= 0
//   E  `substr(0, k + 1)` -> `substr(0, k + 2)`    differs iff the view has a
//                                                  character after its last
//                                                  non-blank -- i.e. iff the
//                                                  message carries a TRAILING
//                                                  BLANK inside its length
//
// Every one of those is a claim about a quantity the corpus supplies, so the
// honest form is a COUNT over the corpus and not an argument about it. Run it
// the way a stub is run -- swapped over
// translations/ControllerBlocks/pitchsaturation.cpp, through
// evidence/PitchSaturation/run_probe.sh -- and read the trailer on stderr.
// VIT Translation Scaffold
// Function: PitchSaturation
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: c9cf442e202f
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-14T21:21:50Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature; the body is transcribed statement for statement. Three statements
// in the reference, three here.
//
// TWO OF THE FIVE ARGUMENTS ARE NEVER READ AND NEVER WRITTEN. `objInst` and
// `DebugVar` are declared INTENT(INOUT) by the reference and appear nowhere in
// its body. They are named in this signature because the SIGNATURE is the
// contract -- the Fortran wrapper passes five things -- and left untouched
// because the reference leaves them untouched. Anything this translation wrote
// into them would be a value the reference does not promise (P6).
//
// THE ERROR-MESSAGE BRANCH IS DEAD IN EVERY SCENARIO THE CAMPAIGN RUNS.
// `coverage/line_coverage.json` records ControllerBlocks.f90:543 --
// `ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)` -- at 0 hits across
// all 27 scenarios, against 391,977 hits on the three statements around it.
// So the kernel and the gate cannot see it, and only the differential harness
// can: `interp1d` sets `aviFAIL = -1` itself when its `xData` is not strictly
// increasing, which is an input the harness generates and the simulation never
// does. Where that branch is measured is recorded in evidence/PitchSaturation/,
// not asserted here.

#include "vit_types.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// ---- the probe's counters, and nothing else changed ----------------------
struct Extremes {
    long long trim_calls = 0, assign_calls = 0;
    int n_min = 1 << 30, n_max = -(1 << 30);
    int cap_min = 1 << 30, cap_max = -(1 << 30);
    size_t size_max = 0;
    long long size_eq_cap = 0;   // A
    long long n_eq_one = 0;      // C
    long long n_le_zero = 0;     // D
    long long trailing_blank = 0;// E
    ~Extremes() {
        std::fprintf(stderr,
            "\nPROBE PitchSaturation over the harness corpus\n"
            "  calls to errmsg_trim                              %lld\n"
            "  calls to assign_errmsg                            %lld\n"
            "  n_ErrMsg seen by errmsg_trim, min .. max           %d .. %d\n"
            "  n_ErrMsg_cap seen by assign_errmsg, min .. max     %d .. %d\n"
            "  longest message offered to assign_errmsg          %zu\n"
            "  A  calls with s.size() == cap                     %lld\n"
            "  C  calls with n_ErrMsg == 1                       %lld\n"
            "  D  calls with n_ErrMsg <= 0                       %lld\n"
            "  E  views with a TRAILING BLANK inside n_ErrMsg    %lld\n",
            trim_calls, assign_calls, n_min, n_max, cap_min, cap_max,
            size_max, size_eq_cap, n_eq_one, n_le_zero, trailing_blank);
    }
};
Extremes g_probe;

// CHARACTER(*), PARAMETER :: RoutineName = 'PitchSaturation'
constexpr std::string_view RoutineName = "PitchSaturation";

// THE TWO HELPERS BELOW ARE COPIED VERBATIM FROM `translations/Functions/
// interp1d.cpp` -- this unit's only callee and the unit that established the
// shape -- with the literal `interp1d` in the two diagnostic strings changed to
// this unit's name. They are not re-derived from the prose (P4). The same code
// stands, character for character modulo that name, in `sigma.cpp`; a shared
// header for it is a change to how every unit in this campaign is built and so
// is X3's question, not this unit's. It is raised in DECISIONS.md instead.

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one. Same shape as
// interp1d, Read_OL_Input and ReadAvrSWAP.
//
// The capacity test is REACHABLE in principle here, as in sigma and unlike
// interp1d: the message written is sixteen characters longer than the one it
// was handed, so an `ErrMsg` arriving within sixteen bytes of the buffer's
// capacity overflows it. Which inputs actually reach it is a question for the
// corpus and not for this comment.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    ++g_probe.assign_calls;
    g_probe.cap_min = std::min(g_probe.cap_min, (int)ErrVar->n_ErrMsg_cap);
    g_probe.cap_max = std::max(g_probe.cap_max, (int)ErrVar->n_ErrMsg_cap);
    g_probe.size_max = std::max(g_probe.size_max, s.size());
    if ((int)s.size() == (int)ErrVar->n_ErrMsg_cap) ++g_probe.size_eq_cap;
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: PitchSaturation: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: PitchSaturation: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// exactly `'PitchSaturation:'`.
// No `== npos` branch: `find_last_not_of` returns `npos`, which is `SIZE_MAX`,
// and `npos + 1` is 0 by the defined wraparound of an unsigned type -- so the
// all-blank string falls out of the same expression as every other.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    ++g_probe.trim_calls;
    g_probe.n_min = std::min(g_probe.n_min, n);
    g_probe.n_max = std::max(g_probe.n_max, n);
    if (n == 1) ++g_probe.n_eq_one;
    if (n <= 0) ++g_probe.n_le_zero;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    if (v.find_last_not_of(' ') + 1 < v.size()) ++g_probe.trailing_blank;
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

double PitchSaturation(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
                       objectinstances_t* objInst, debugvariables_t* DebugVar,
                       errorvariables_view_t* ErrVar) {
    // The reference does not name these two; see the header. Voided so that the
    // signature can carry them without a warning suppressing itself into a
    // habit -- the day one of them IS read, the void goes away with the read.
    (void)objInst;
    (void)DebugVar;

    // ! Define minimum blade pitch angle for peak shaving as a function of
    // ! estimated wind speed
    // LocalVar%PS_Min_Pitch = interp1d(CntrPar%PS_WindSpeeds,
    //                                  CntrPar%PS_BldPitchMin,
    //                                  LocalVar%WE_Vw_F, ErrVar)
    //
    // The two extents are passed independently because the C signature takes
    // them independently, and that is exactly the disagreement interp1d's own
    // first branch reports: its reference ABORTS on `SIZE(xData) /= SIZE(yData)`
    // (evidence/interp1d/reference.size-mismatch-aborts.txt) and the translation
    // continues. Here the two arrays are allocated together from one count in
    // `ReadControlParameterFileSub`, so the Fortran cannot produce a mismatch at
    // this call site; the harness can, and `harness/ranges.toml` is where that
    // is tied if it costs anything.
    //
    // `PS_Min_Pitch` is a scalar field of an INOUT view argument, so it does
    // not travel back through a C_LOC'd buffer the way an allocatable array
    // does: the integration needs `--reverse-copy`.
    LocalVar->PS_Min_Pitch =
        interp1d_c(CntrPar->PS_WindSpeeds, CntrPar->n_PS_WindSpeeds,
                   CntrPar->PS_BldPitchMin, CntrPar->n_PS_BldPitchMin,
                   LocalVar->WE_Vw_F, ErrVar);

    // ! Total min pitch limit is greater of peak shaving and power control pitch
    // PitchSaturation = max(LocalVar%PS_Min_Pitch, LocalVar%PRC_Min_Pitch)
    //
    // `std::max(a, b)` is `(a < b) ? b : a` and Fortran's two-argument MAX for
    // REAL(8) is the same selection in the same operand order, so the two agree
    // on every pair including the signed zeros. Written as the intrinsic rather
    // than as a hand-rolled conditional deliberately: a `? :` here would offer a
    // `compare_op` mutant whose kill would prove only that the corpus contains a
    // pair on the boundary, and would read as arithmetic the reference does not
    // have. What it costs is that the ORDER of the two operands gets no mutant
    // from `swap_call_args` that is not equivalent; that is measured by hand in
    // evidence/PitchSaturation/ rather than left implied.
    const double PitchSaturation_result =
        std::max(LocalVar->PS_Min_Pitch, LocalVar->PRC_Min_Pitch);

    // ! Add RoutineName to error message
    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    // ENDIF
    //
    // Reached only when the `interp1d` call above set `aviFAIL = -1`, or when
    // the caller arrived with it already negative. Dead in all 27 scenarios --
    // see the header.
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }

    // The reference assigns the result BEFORE the error block and the error
    // block does not touch it, so returning here is the same program. The
    // assignment is kept where the reference has it rather than folded into
    // this return, so that the two statements stay one-to-one.
    return PitchSaturation_result;
}
