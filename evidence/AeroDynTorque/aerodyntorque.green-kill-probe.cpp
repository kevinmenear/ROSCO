// DOES A GREEN CASE THAT KILLS THIS MUTANT EXIST AT ALL?
//
// The mutation census answered a weaker question -- "does the 1131-case corpus
// contain the killing input" -- and answered it by counting. That is the right
// question only if the answer is "no, and here is the input". For two of unit
// #48's four survivors the answer is stronger and worse: NO INPUT EXISTS on
// which this harness could kill them, because every input that separates the
// mutant from the original ALSO separates the ORIGINAL from the harness's own
// reference. A corpus cannot contain a case that is red before the mutation.
//
// So this probe asks the decisive question directly, for each survivor:
//
//     is there an (entry ErrMsg, entry n_ErrMsg, staging capacity, callee
//     message) at which  ORIGINAL == REFERENCE  and  MUTANT != REFERENCE ?
//
// A `yes` is disposition (b) -- fix the inputs; the configuration is printed.
// A `no` over an exhaustive sweep of the domain in which the two can differ at
// all is disposition (c) -- a blind spot of the instrument, not of the corpus.
//
// ------------------------------------------------------- the two chains
//
// REFERENCE (the harness's `ref` side, on a clean tree): Fortran AeroDynTorque
// -> Fortran interp2d -> Fortran interp1d, `ErrVar%ErrMsg` a reallocating
// `CHARACTER(:), ALLOCATABLE` throughout, and exactly ONE capacity gate: the
// generated bridge's write-back, on the final message.
//
//     msg_ref = "AeroDynTorque:" + TRIM(M)          M = what the callee left
//     if |msg_ref| > cap   the export is REFUSED and the buffer keeps the
//                          value it had ON ENTRY
//
// TRANSLATION: C++ AeroDynTorque -> the `interp2d_c` bridge -> the same Fortran
// interp2d, with TWO gates -- the bridge's write-back on M, then
// `assign_errmsg` on the final message.
//
//     if |M| <= cap   the buffer becomes M, n_ErrMsg = |M|;  else UNCHANGED
//     t = errmsg_trim(buffer, n_ErrMsg)
//     if 14 + |t| <= cap   the buffer becomes "AeroDynTorque:" + t;  else UNCHANGED
//
// This is the composition finding of `harness.staging_composition.txt`, written
// as executable arithmetic instead of prose.
//
// ------------------------------------------------------- what M can be
//
// M is NOT free and it is NOT a function of the entry length either. Read off
// the clean reference, every exit of `interp2d` on which `aviFAIL < 0` holds
// passes through exactly one reallocating assignment `RoutineName//':'//TRIM(
// ErrVar%ErrMsg)` -- interp1d's on the five early RETURNs, interp2d's own on
// the bilinear fall-through -- and what that assignment TRIMS is either the
// unit's own entry message or a message the callee REPLACED it with first:
//
//   interp2d:  ErrVar%ErrMsg = ' xData is not strictly increasing'   (33 ch)
//              ErrVar%ErrMsg = ' yData is not strictly increasing'   (33 ch)
//   interp1d:  ErrVar%ErrMsg = ' xData is not strictly increasing'   (33 ch)
//
// The two SIZE-mismatch arms of each are excluded from the admissible domain by
// `harness/ranges.toml` (`PerfData_Cp_mat_cols = same_as PerfData_Beta_vec_n`
// and rows/TSR): their `WRITE` formats a 57-character record into `ErrMsg` AS
// IT STANDS and gfortran ends the process with `End of record`. That judgement
// is committed and is not weakened here.
//
// So M ranges over exactly:
//
//     "interp1d:" + TRIM(entry)            9 + t
//     "interp2d:" + TRIM(entry)            9 + t
//     "interp1d: xData is not strictly increasing"     42
//     "interp2d: yData is not strictly increasing"     42
//
// and the 42-byte forms are the ones the first model of this probe
// (`aerodyntorque.trim-composition-probe.cpp`) missed by tying M's length to
// the entry's: with M = 9 + t the three trim mutants are unreachable, and with
// M = 42 two of them are not. Recorded rather than deleted -- the first model
// gave the wrong answer for a reason worth keeping.
//
// ------------------------------------------------------------- CONTROLS
//
// Printed, not left to the reader:
//   NEGATIVE  the ORIGINAL against itself must find 0 kills at every
//             configuration -- if it finds one the comparison is broken.
//   POSITIVE  88466711 must be found killable, at cap == the final message
//             length. That is the case `harness.staging_composition.txt`
//             predicted (cap 30 for the corpus's own base case) and it is what
//             `--disable R13_staging_capacity` removed.
//   RED-BEFORE-MUTATION  for each mutant, the count of configurations where the
//             ORIGINAL is already red is printed beside the kill count, because
//             "no green kill exists" means nothing without it.
//
// Build and run:
//   docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
//     g++ -O2 -ffp-contract=off -std=c++17 \
//     evidence/AeroDynTorque/aerodyntorque.green-kill-probe.cpp \
//     -o /tmp/greenkill && /tmp/greenkill"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr int BUFCAP = 512;      // the real allocation; `cap` is the STATED capacity
constexpr int MAXCAP = 96;       // capacities swept, 0 .. MAXCAP

// ---- the two helpers, from translations/Functions/aerodyntorque.cpp, with the
// four mutated tokens as template knobs so that "one token apart" holds by
// construction. The shipped values are <0, 0, 0, 1>.
template <int CAP_GE, int TRIM_GUARD, int TRIM_ELSE, int TRIM_PLUS>
std::string errmsg_trim(const char* buf, int n) {
    const std::string_view v(buf, n > TRIM_GUARD ? static_cast<size_t>(n)
                                                 : static_cast<size_t>(TRIM_ELSE));
    return std::string(v.substr(0, v.find_last_not_of(' ') + TRIM_PLUS));
}

template <int CAP_GE, int TRIM_GUARD, int TRIM_ELSE, int TRIM_PLUS>
void assign_errmsg(char* buf, int& n, int cap, std::string_view s) {
    const bool refuse = CAP_GE ? (static_cast<int>(s.size()) >= cap)
                               : (static_cast<int>(s.size()) >  cap);
    if (refuse) return;
    std::memcpy(buf, s.data(), s.size());
    n = static_cast<int>(s.size());
}

// The whole translation chain: the callee's staged write-back, then the tail.
template <int CAP_GE, int TRIM_GUARD, int TRIM_ELSE, int TRIM_PLUS>
void translation(char* buf, int& n, int cap, const std::string& M) {
    if (static_cast<int>(M.size()) <= cap) {          // the bridge's write-back
        std::memcpy(buf, M.data(), M.size());
        n = static_cast<int>(M.size());
    }
    const std::string t = errmsg_trim<CAP_GE, TRIM_GUARD, TRIM_ELSE, TRIM_PLUS>(buf, n);
    assign_errmsg<CAP_GE, TRIM_GUARD, TRIM_ELSE, TRIM_PLUS>(
        buf, n, cap, std::string("AeroDynTorque:") + t);
}

// The reference chain: unbounded inside, one gate on the way out.
std::string rtrim(const std::string& s) {
    size_t p = s.find_last_not_of(' ');
    return p == std::string::npos ? std::string() : s.substr(0, p + 1);
}
void reference(char* buf, int& n, int cap, const std::string& M) {
    const std::string f = "AeroDynTorque:" + rtrim(M);
    if (static_cast<int>(f.size()) > cap) return;     // the export, REFUSED
    std::memcpy(buf, f.data(), f.size());
    n = static_cast<int>(f.size());
}

// ---- the input space ------------------------------------------------------
const char* const SHAPES[] = {
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345",   // no blank anywhere
    "AB CD EF GH IJ KL MN OPQ RS TUVW",   // embedded blanks, non-blank tail
    "ABCDEFGHIJ                      ",   // trailing blanks
    "                                ",   // all blanks
    " A                              ",   // one non-blank, long blank tail
};
constexpr int N_SHAPES = sizeof(SHAPES) / sizeof(SHAPES[0]);
const char FILLS[] = {'\0', ' ', 'Z'};    // bytes outside [0, n)
constexpr int N_FILLS = sizeof(FILLS) / sizeof(FILLS[0]);

const char* const ERRTEXT[] = {
    " xData is not strictly increasing",
    " yData is not strictly increasing",
};
const char* const PREFIX[] = {"interp1d:", "interp2d:"};

struct Config { int shape, fill, n, cap, mform; };
// mform 0,1 : PREFIX[k] + TRIM(entry)      2,3 : PREFIX[0/1] + ERRTEXT[0/1]

void setup(char* buf, const Config& c) {
    std::memset(buf, FILLS[c.fill], BUFCAP);
    const int keep = c.n > 0 ? c.n : 0;
    const int have = static_cast<int>(std::strlen(SHAPES[c.shape]));
    for (int i = 0; i < keep && i < BUFCAP; ++i)
        buf[i] = i < have ? SHAPES[c.shape][i] : FILLS[c.fill];
}

// What the callee left, as a Fortran string. The callee is UNMUTATED: it is a
// different unit's code, reached through a bridge.
std::string callee_message(const char* buf, const Config& c) {
    const int keep = c.n > 0 ? c.n : 0;
    switch (c.mform) {
        case 0: return std::string(PREFIX[0]) + rtrim(std::string(buf, keep));
        case 1: return std::string(PREFIX[1]) + rtrim(std::string(buf, keep));
        case 2: return std::string(PREFIX[0]) + ERRTEXT[0];
        default: return std::string(PREFIX[1]) + ERRTEXT[1];
    }
}

struct Variant { const char* id; const char* site; void (*run)(char*, int&, int, const std::string&); };
void run_orig(char* b, int& n, int c, const std::string& M) { translation<0,0,0,1>(b,n,c,M); }
void run_8846(char* b, int& n, int c, const std::string& M) { translation<1,0,0,1>(b,n,c,M); }
void run_11c1(char* b, int& n, int c, const std::string& M) { translation<0,1,0,1>(b,n,c,M); }
void run_06f7(char* b, int& n, int c, const std::string& M) { translation<0,0,1,1>(b,n,c,M); }
void run_2602(char* b, int& n, int c, const std::string& M) { translation<0,0,0,2>(b,n,c,M); }

// The harness compares the STATED CAPACITY's worth of bytes plus the extent --
// `harness.staging_composition.txt` records "case 1140 compared 16 bytes" at a
// capacity of 16. Both readings are computed; a disagreement between them would
// itself be a finding, so it is printed rather than chosen between.
bool same_bytes(const char* a, const char* b, int n_a, int n_b, int cap) {
    return n_a == n_b && std::memcmp(a, b, cap) == 0;
}
bool same_extent_only(const char* a, const char* b, int n_a, int n_b) {
    const int la = n_a > 0 ? n_a : 0;
    return n_a == n_b && std::memcmp(a, b, la) == 0;
}

}  // namespace

int main() {
    const Variant vars[] = {
        {"88466711", "assign_errmsg  '>' -> '>='",        run_8846},
        {"11c1e326", "errmsg_trim    'n > 0' -> 'n > 1'", run_11c1},
        {"06f7d2c8", "errmsg_trim    ': 0' -> ': 1'",     run_06f7},
        {"26021804", "errmsg_trim    '+ 1' -> '+ 2'",     run_2602},
    };

    std::vector<Config> configs;
    for (int s = 0; s < N_SHAPES; ++s)
      for (int f = 0; f < N_FILLS; ++f)
        for (int n = -2; n <= 32; ++n)
          for (int cap = 0; cap <= MAXCAP; ++cap)
            for (int m = 0; m < 4; ++m)
              configs.push_back({s, f, n, cap, m});

    std::printf("GREENKILL configs=%zu  shapes=%d fills=%d n=[-2,32] cap=[0,%d] "
                "M-forms=4\n", configs.size(), N_SHAPES, N_FILLS, MAXCAP);
    std::printf("M-forms: 0 'interp1d:'+TRIM(entry)  1 'interp2d:'+TRIM(entry)  "
                "2/3 the 42-byte 'not strictly increasing' messages\n\n");
    std::printf("%-10s %-34s %9s %9s %9s  %s\n", "mutant", "site",
                "GREENKILL", "orig-red", "kill-any", "first green kill");
    std::printf("%-10s %-34s %9s %9s %9s  %s\n", "----------",
                "----------------------------------", "---------", "---------",
                "---------", "----------------------------------------");

    int rc = 0;
    long greenkill_total = 0;
    long extent_disagreements = 0;

    for (const Variant& var : vars) {
        long green = 0, origred = 0, killany = 0;
        Config first{}; bool have = false;
        for (const Config& c : configs) {
            char e[BUFCAP], r[BUFCAP], o[BUFCAP], m[BUFCAP];
            setup(e, c);
            std::memcpy(r, e, BUFCAP); std::memcpy(o, e, BUFCAP); std::memcpy(m, e, BUFCAP);
            const std::string M = callee_message(e, c);
            int nr = c.n, no = c.n, nm = c.n;
            reference(r, nr, c.cap, M);
            run_orig(o, no, c.cap, M);
            var.run(m, nm, c.cap, M);

            const bool orig_ok = same_bytes(o, r, no, nr, c.cap);
            const bool mut_ok  = same_bytes(m, r, nm, nr, c.cap);
            if (same_bytes(o, m, no, nm, c.cap) != same_extent_only(o, m, no, nm))
                ++extent_disagreements;
            if (!orig_ok) ++origred;
            if (!mut_ok && orig_ok) { ++green; if (!have) { first = c; have = true; } }
            if (!same_bytes(o, m, no, nm, c.cap)) ++killany;
        }
        greenkill_total += green;
        std::printf("%-10s %-34s %9ld %9ld %9ld  ", var.id, var.site, green, origred, killany);
        if (have) {
            char e[BUFCAP]; setup(e, first);
            const std::string M = callee_message(e, first);
            std::printf("shape=%d fill=%d n=%d cap=%d |M|=%d\n",
                        first.shape, first.fill, first.n, first.cap, (int)M.size());
        } else {
            std::printf("(NONE EXISTS)\n");
        }
    }

    // NEGATIVE CONTROL: the original against the reference, scored as if it were
    // a mutant of itself. Must be zero.
    long self = 0;
    for (const Config& c : configs) {
        char e[BUFCAP], r[BUFCAP], o[BUFCAP];
        setup(e, c); std::memcpy(r, e, BUFCAP); std::memcpy(o, e, BUFCAP);
        const std::string M = callee_message(e, c);
        int nr = c.n, no = c.n;
        reference(r, nr, c.cap, M);
        run_orig(o, no, c.cap, M);
        const bool ok = same_bytes(o, r, no, nr, c.cap);
        char o2[BUFCAP]; std::memcpy(o2, e, BUFCAP); int no2 = c.n;
        run_orig(o2, no2, c.cap, M);
        if (ok && !same_bytes(o2, r, no2, nr, c.cap)) ++self;
    }
    std::printf("\nCONTROL negative  the ORIGINAL scored against itself: %ld "
                "green kill(s) -- %s\n", self, self ? "COMPARISON IS BROKEN" : "as required");
    if (self) rc = 1;

    std::printf("CONTROL positive  a green kill was found for %ld of the four "
                "mutant(s); the probe %s\n",
                greenkill_total ? 1L : 0L,
                greenkill_total ? "can report a kill" : "IS BROKEN");
    if (!greenkill_total) rc = 1;

    std::printf("CONTROL reading   full-capacity and extent-only comparison "
                "disagree at %ld configuration(s)\n", extent_disagreements);
    return rc;
}
