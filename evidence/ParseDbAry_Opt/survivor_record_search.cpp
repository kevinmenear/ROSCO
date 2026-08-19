// SEARCH FOR A RECORD THAT DISTINGUISHES A SURVIVING MUTANT FROM THE SHIPPED
// TRANSLATION -- the question the mutation score asks, taken off the corpus and
// asked directly of the two programs.
//
// WHY. A surviving mutant is one of three things: equivalent, unreached by the
// corpus, or a blind spot. The mutation sweep cannot tell them apart, because
// its answer for all three is the same word. `run_survivor_record_search.sh`
// builds this file once against the SHIPPED translation and once against each
// surviving mutant, runs both over the same record space, and diffs:
//
//   a record where they differ  -> NOT equivalent. The record is the corpus
//                                  lever, named, and no further argument is
//                                  needed.
//   no record in the space      -> evidence toward equivalence, bounded BY THE
//                                  SPACE, which is printed so the bound is
//                                  readable rather than implied.
//
// IT IS NOT A DECLARATION AND IT IS NOT FOLDED INTO THE SCORE. It compares the
// translation against ITSELF-MUTATED, so it says nothing about whether either
// agrees with the Fortran reference; that is the differential harness's job and
// this cannot substitute for it. What it settles is which of the three cases a
// survivor is in.
//
// THE RECORD SPACE, and every part of it is here because a survivor needs it:
//
//   HEAD   every string of length 1..3 over a 24-character alphabet, at the
//          START of the record with blanks after -- the shape every corpus
//          record has today.
//   TAIL   the same strings at the END, so the last character of the token is
//          the last byte of the record. This is the shape `_RECORD_TAILS`
//          introduced and it is where five of the third dispatch's survivors
//          lived.
//   LONG   crafted full-width records: the four tail forms the corpus now
//          carries, plus the ones it does not -- a record ending inside an
//          exponent, a 62-character NaN payload, a repeat count with a `;`.
//
// Textual include, for the reason `print_conformance.cpp` states: the parser
// lives in an anonymous namespace and a copy here would go stale silently.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -I<test dir> \
//              -DVIT_TRANSLATION='"<path to a .cpp>"' \
//              survivor_record_search.cpp -o search -lgfortran
//   run:   ./search            > one line per record

#include <ISO_Fortran_binding.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "parsedbary_opt_callees.h"

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) {
    std::fprintf(stderr, "survivor_record_search: findline_c reached\n");
    std::abort();
}
void getwords_c(char*, int, char*, int, int) {
    std::fprintf(stderr, "survivor_record_search: getwords_c reached\n");
    std::abort();
}
void int2lstr_c(int, char*) {
    std::fprintf(stderr, "survivor_record_search: int2lstr_c reached\n");
    std::abort();
}
}

#include VIT_TRANSLATION

namespace {

// The alphabet is every character the parser's own predicates test, plus the
// letters of `inf` / `infinity` / `nan` and their uppercase forms, plus one
// character (`x`) that is in none of them so a "matches nothing" case exists.
const char ALPHABET[] = "0123456789+-.eEdDqQnNaAiIfFtTyYxX ,;/*()";

// The sentinel is the one the earlier probes use, so an element the transfer
// never reached is visible as itself rather than as a plausible zero.
const double SENTINEL = -987.654;

void report(const char* tag, const std::string& token, const std::string& rec) {
    std::vector<char> buf(rec.begin(), rec.end());
    double v[3] = {SENTINEL, SENTINEL, SENTINEL};
    const int ios = list_read_reals(buf.data(), MaxLineLength, v, 3);
    std::uint64_t b[3];
    for (int i = 0; i < 3; ++i) std::memcpy(&b[i], &v[i], sizeof b[i]);
    std::printf("%s|%s|%d|%016llX|%016llX|%016llX\n", tag, token.c_str(), ios,
                (unsigned long long)b[0], (unsigned long long)b[1],
                (unsigned long long)b[2]);
}

std::string head(const std::string& tok) {
    std::string r(static_cast<std::size_t>(MaxLineLength), ' ');
    std::memcpy(&r[0], tok.data(), tok.size());
    return r;
}

std::string tail(const std::string& tok) {
    std::string r(static_cast<std::size_t>(MaxLineLength), ' ');
    std::memcpy(&r[r.size() - tok.size()], tok.data(), tok.size());
    return r;
}

std::string rep(char c, int n) { return std::string(static_cast<std::size_t>(n), c); }

}  // namespace

int main() {
    const std::size_t A = std::strlen(ALPHABET);

    for (std::size_t i = 0; i < A; ++i) {
        const std::string t(1, ALPHABET[i]);
        report("HEAD", t, head(t));
        report("TAIL", t, tail(t));
    }
    for (std::size_t i = 0; i < A; ++i)
        for (std::size_t j = 0; j < A; ++j) {
            std::string t;
            t += ALPHABET[i];
            t += ALPHABET[j];
            report("HEAD", t, head(t));
            report("TAIL", t, tail(t));
        }
    for (std::size_t i = 0; i < A; ++i)
        for (std::size_t j = 0; j < A; ++j)
            for (std::size_t k = 0; k < A; ++k) {
                std::string t;
                t += ALPHABET[i];
                t += ALPHABET[j];
                t += ALPHABET[k];
                report("HEAD", t, head(t));
                report("TAIL", t, tail(t));
            }

    // The full-width records: the four the corpus now carries, and the ones it
    // does not. Each is exactly MaxLineLength characters, so the record ends
    // inside the value rather than in padding.
    const int L = MaxLineLength;
    const std::pair<const char*, std::string> LONG[] = {
        {"tail:digits", rep('0', L - 1) + "1"},
        {"tail:frac", "1." + rep('0', L - 3) + "5"},
        {"tail:word", rep('0', L - 4) + ",inf"},
        {"tail:wordUC", rep('0', L - 4) + ",nAn"},
        {"tail:expE", rep('0', L - 3) + "1E"},
        {"tail:expEsign", rep('0', L - 4) + "1E+"},
        {"tail:expEdig", rep('0', L - 5) + "1E+2"},
        {"tail:nanpay", rep('0', L - 63) + "," + "nan(" + rep('7', 57) + ")"},
        {"tail:repeat", rep('0', L - 10) + ",2*1.5;3.0"},
        {"tail:dotexp", rep('0', L - 3) + ",.e"},
        {"head:repeat", "2*1.5;3.0"},
        {"head:dotexp", ".e 1.0"},
        {"head:nanpay", std::string("nan(") + rep('7', 57) + ")"},
    };
    for (const auto& r : LONG) {
        if (r.second.size() > static_cast<std::size_t>(L)) {
            std::fprintf(stderr, "survivor_record_search: %s is %zu characters, "
                                 "longer than the %d-byte record\n",
                         r.first, r.second.size(), L);
            return 2;
        }
        std::string rec = r.second;
        rec.resize(static_cast<std::size_t>(L), ' ');
        report("LONG", r.first, rec);
    }
    return 0;
}
