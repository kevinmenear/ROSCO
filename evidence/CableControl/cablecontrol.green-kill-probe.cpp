// THE GREEN-KILL PROBE, unit #49 `CableControl`.
//
// THE QUESTION IT ANSWERS, and it is not the one the census answers.
// `errmsg_census.txt` asks "does the SCORED CORPUS contain the input that kills
// this mutant" and answers 0, 0 and 0. That number cannot tell
//
//     (b) the corpus does not contain the killing input, and a corpus could
//     (c) NO corpus can, because every separating input is already RED
//
// apart, and the campaign has had both: unit #48 recorded four survivors with
// reading (b) and two of them had (c). The question that decides it is
//
//     does any input exist with  ORIGINAL == REFERENCE  and  MUTANT != REFERENCE ?
//
// because a corpus can only kill where the original is green.
//
// SO THIS EXECUTES BOTH CHAINS. Not a proof from an inequality -- unit #48's
// first model of exactly this probe proved three mutants behaviour-preserving
// from a clean argument and was FALSE, and it was caught by running it.
//
//   the REFERENCE chain   the generated Fortran bridge's import
//                         (cablecontrol_bridge.f90:1758-1764), then
//                         `ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)`,
//                         then the bridge's export with its ONE capacity gate,
//                         `IF (LEN(...) <= cap)` (:3265-3277)
//   the TRANSLATION chain `errmsg_trim` then `assign_errmsg`, with its own gate
//                         `if (s.size() > cap) refuse`
//
// Both are transcribed from those two files, side by side, in the two functions
// below. The Fortran semantics that matter and where each comes from:
//
//   * the import is `IF (cap > 0) ALLOCATE(CHARACTER(LEN=n) :: ErrMsg)`, so
//     `cap == 0` leaves the field UNALLOCATED and `n < 0` allocates a
//     ZERO-LENGTH string (a negative LEN in ALLOCATE is length zero, F2018
//     9.7.1.2). That is the same collapse the view's VIT_CHAR_UNALLOCATED
//     convention makes on the C side, and it is why `n < 0` and `n == 0` are
//     one case for the reference and two for one of the mutants.
//   * the export leaves BOTH the buffer and `n` unchanged when the message does
//     not fit, which is the "refuse rather than truncate" convention unit #23
//     established -- the translation's `assign_errmsg` does the same.
//   * `<=` in the bridge and `>` in the translation are the SAME boundary
//     written from the two sides. That is the thing mutant 8d2724ea moves.
//
// THE CONTROLS. A negative control is the original scored against itself, which
// must be 0 -- a probe that reports kills for a mutant that is not one is
// measuring its own model and not the program. A positive control is a mutant
// the sweep already KILLED (`ErrVar->aviFAIL < 0` negated, killed at 3336 of
// 3354), which must be non-zero.
//
//   docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
//       g++ -O2 -std=c++17 -o /tmp/cc_gkp \
//           evidence/CableControl/cablecontrol.green-kill-probe.cpp && /tmp/cc_gkp"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr std::string_view RoutineName = "StructuralControl";

// The five chains under test. ORIGINAL is the shipped translation; M1..M3 are
// the three survivors of mutation/CableControl.json, each named by its id.
enum Variant { ORIGINAL, M1_8d2724ea, M2_97aefd80, M3_1a26a963, CTRL_avifail };

// The staging buffer as the emitted test builds it: `std::vector<char>` of
// `cap` bytes, ZERO-FILLED, with the case's own message in the first `n`.
// (cablecontrol_test.cpp:1296 -- `std::vector<char> ErrVar_ErrMsg_a(..., 0)`.)
struct Buf {
    std::vector<char> bytes;
    int n;
    bool operator==(const Buf& o) const { return n == o.n && bytes == o.bytes; }
};

// ---------------------------------------------------------------- reference --
// cablecontrol_bridge.f90, import -> body -> export.
Buf reference(const Buf& in, int aviFAIL) {
    Buf out = in;
    const int cap = static_cast<int>(in.bytes.size());

    // IF (vit_ErrVar_ErrMsg_cap > 0) ALLOCATE(CHARACTER(LEN=n) :: ErrMsg)
    bool allocated = cap > 0;
    // A negative LEN allocates a zero-length string.
    int flen = in.n > 0 ? in.n : 0;
    std::string f(in.bytes.data(), static_cast<size_t>(flen < cap ? flen : cap));

    // IF (ErrVar%aviFAIL < 0) ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    if (aviFAIL < 0 && allocated) {
        size_t t = f.find_last_not_of(' ');
        std::string trimmed = (t == std::string::npos) ? std::string() : f.substr(0, t + 1);
        f = std::string(RoutineName) + ':' + trimmed;   // reallocating assignment
    }

    // IF (ALLOCATED(...)) THEN; IF (LEN(...) <= cap) THEN <copy LEN, set n>
    if (allocated && static_cast<int>(f.size()) <= cap) {
        std::memcpy(out.bytes.data(), f.data(), f.size());
        out.n = static_cast<int>(f.size());
    }
    return out;
}

// -------------------------------------------------------------- translation --
// translations/Controllers/cablecontrol.cpp, errmsg_trim + assign_errmsg.
Buf translation(const Buf& in, int aviFAIL, Variant v) {
    Buf out = in;
    const int cap = static_cast<int>(in.bytes.size());
    const char* p = in.bytes.empty() ? nullptr : in.bytes.data();

    const bool guard = (v == CTRL_avifail) ? !(aviFAIL < 0) : (aviFAIL < 0);
    if (!guard) return out;

    // errmsg_trim
    const int n = in.n;
    size_t len;
    if (v == M2_97aefd80)      len = n > 1 ? static_cast<size_t>(n) : 0;   // `n > 0` -> `n > 1`
    else if (v == M3_1a26a963) len = n > 0 ? static_cast<size_t>(n) : 1;   // `: 0`   -> `: 1`
    else                       len = n > 0 ? static_cast<size_t>(n) : 0;
    if (p == nullptr) len = 0;
    if (len > in.bytes.size()) len = in.bytes.size();
    const std::string_view view(p, len);
    const std::string trimmed(view.substr(0, view.find_last_not_of(' ') + 1));

    const std::string s = std::string(RoutineName) + ':' + trimmed;

    // assign_errmsg
    if (p == nullptr) return out;
    const bool refuse = (v == M1_8d2724ea)
                            ? static_cast<int>(s.size()) >= cap    // `>` -> `>=`
                            : static_cast<int>(s.size()) > cap;
    if (refuse) return out;
    std::memcpy(out.bytes.data(), s.data(), s.size());
    out.n = static_cast<int>(s.size());
    return out;
}

// The entry messages the corpus's own character ladder produces, plus the two
// shapes it does NOT: an all-blank body and a body whose bytes are ordinary.
const char* FILLS[] = {"", "x", "xy", "abcdefgh", "abcdefghijklm",
                       " ", "  ", "x ", " x", "xyz     ", "\0abc"};

}  // namespace

int main() {
    struct Row { const char* name; Variant v; };
    const Row rows[] = {
        {"NEGATIVE CONTROL (original vs itself)", ORIGINAL},
        {"8d2724ea  `s.size() > cap` -> `>=`   ", M1_8d2724ea},
        {"97aefd80  `n > 0 ? n : 0` -> `n > 1` ", M2_97aefd80},
        {"1a26a963  `n > 0 ? n : 0` -> `: 1`   ", M3_1a26a963},
        {"POSITIVE CONTROL (aviFAIL guard neg.)", CTRL_avifail},
    };

    long configs = 0;
    long kills[5] = {0, 0, 0, 0, 0};
    long seps[5] = {0, 0, 0, 0, 0};
    long redcases = 0;
    char witness[5][160] = {{0}, {0}, {0}, {0}, {0}};

    for (int fi = 0; fi < 11; ++fi) {
        const char* fill = FILLS[fi];
        const size_t flen = (fi == 10) ? 4 : std::strlen(fill);
        for (int n = -2; n <= 32; ++n) {
            for (int cap = 0; cap <= 64; ++cap) {
                for (int avi : {-1, 0, 1, 2}) {
                    Buf in;
                    in.bytes.assign(static_cast<size_t>(cap), '\0');
                    for (size_t i = 0; i < flen && i < in.bytes.size(); ++i)
                        in.bytes[i] = fill[i];
                    in.n = n;
                    ++configs;

                    const Buf ref = reference(in, avi);
                    const Buf org = translation(in, avi, ORIGINAL);
                    const bool green = (org == ref);
                    if (!green) ++redcases;

                    for (int r = 0; r < 5; ++r) {
                        const Buf mut = translation(in, avi, rows[r].v);
                        if (!(mut == org)) ++seps[r];
                        if (green && !(mut == ref)) {
                            if (kills[r] == 0)
                                std::snprintf(witness[r], sizeof witness[r],
                                    "fill=%d(\"%s\") n=%d cap=%d aviFAIL=%d  "
                                    "ref.n=%d org.n=%d mut.n=%d",
                                    fi, fill, n, cap, avi, ref.n, org.n, mut.n);
                            ++kills[r];
                        }
                    }
                }
            }
        }
    }

    std::printf("GREEN-KILL PROBE -- unit #49 CableControl\n");
    std::printf("========================================\n\n");
    std::printf("  configurations swept                              %ld\n", configs);
    std::printf("  of those, RED before any mutation (org != ref)    %ld\n\n", redcases);
    std::printf("  %-38s %12s %12s\n", "chain", "separates", "GREEN kills");
    for (int r = 0; r < 5; ++r)
        std::printf("  %-38s %12ld %12ld\n", rows[r].name, seps[r], kills[r]);
    std::printf("\n  THE SMALLEST WITNESS FOR EACH, printed by the probe rather than argued:\n");
    for (int r = 0; r < 5; ++r)
        if (kills[r]) std::printf("    %-38s %s\n", rows[r].name, witness[r]);
    std::printf("\n  A GREEN kill is an input with ORIGINAL == REFERENCE and\n");
    std::printf("  MUTANT != REFERENCE -- i.e. an input a green corpus could hold.\n");
    return 0;
}
