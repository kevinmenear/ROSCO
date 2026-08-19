// SEARCH FOR A RECORD THAT DISTINGUISHES A SURVIVING MUTANT FROM THE SHIPPED
// TRANSLATION -- the question the mutation score asks, taken off the corpus and
// asked directly of the two programs.
//
// COPIED from `evidence/ParseDbAry_Opt/survivor_record_search.cpp` (P4) with
// three things changed and everything else left alone: the reader is
// `list_read_ints` and the elements are `int32_t`, so they print as decimal
// rather than as IEEE bit patterns; the LONG record set is this unit's own
// boundaries rather than the REAL parser's; and the build carries
// `-fsanitize=address,undefined`. The last is not cosmetic -- see below.
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
// WHY THE SANITISER IS ON HERE AND NOT ON THE SIBLING'S. Ten of this unit's
// twenty-six survivors are `p < len` -> `p <= len` and `rec[p]` -> `rec[p + 1]`
// inside the separator scans. The difference they make is a read of `rec[2048]`
// -- one past the end of the `std::vector<char>(2048)` the record lives in --
// and on this allocator the byte that follows is very often a blank, so the two
// programs COMPUTE THE SAME ANSWER and a value comparison sees nothing. That is
// the whole reason `vit_mutate --sanitize` exists, and a search that did not
// carry it would report `none` for exactly the class it was built to reach.
// The baseline running CLEAN under the same flags is the control: if the
// shipped translation reported, every mutant would "differ" by inheriting it.
//
// IT IS NOT A DECLARATION AND IT IS NOT FOLDED INTO THE SCORE. It compares the
// translation against ITSELF-MUTATED, so it says nothing about whether either
// agrees with the Fortran reference; that is the differential harness's job and
// this cannot substitute for it. What it settles is which of the three cases a
// survivor is in.
//
// THE RECORD SPACE, and every part of it is here because a survivor needs it:
//
//   HEAD   every string of length 1..3 over a 40-character alphabet, at the
//          START of the record with blanks after -- the shape every corpus
//          record had before `_RECORD_TAILS`. It is ALSO the shape in which a
//          separator scan runs all the way to column 2048, because there is
//          nothing to its right: no key word, which every record FindLine
//          matches necessarily has.
//   TAIL   the same strings at the END, so the last character of the token is
//          the last byte of the record.
//   LONG   crafted full-width records: the four tail forms the corpus now
//          carries, plus the ones it does not -- a repeat count that runs to
//          the boundary, a semicolon after a blank at the boundary, and the
//          repeat-count ceiling either side.
//
// Textual include, for the reason `parser_conformance.cpp` states: the parser
// lives in an anonymous namespace and a copy here would go stale silently.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -fsanitize=address,undefined \
//              -I<test dir> -DVIT_TRANSLATION='"<path to a .cpp>"' \
//              survivor_record_search.cpp -o search -lgfortran
//   run:   ./search            > one line per record

#include <ISO_Fortran_binding.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "parseinary_opt_callees.h"

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
// letters of `inf` / `infinity` / `nan` and their uppercase forms (which an
// INTEGER read rejects, and rejecting them is a branch), plus one character
// (`x`) that is in none of them so a "matches nothing" case exists. Kept
// character for character from the sibling's so the two spaces are comparable.
const char ALPHABET[] = "0123456789+-.eEdDqQnNaAiIfFtTyYxX ,;/*()";

// The sentinel is the one this unit's Fortran probes use, so an element the
// transfer never reached is visible as itself rather than as a plausible zero.
const std::int32_t SENTINEL = -987654;

void report(const char* tag, const std::string& token, const std::string& rec) {
    std::vector<char> buf(rec.begin(), rec.end());
    std::int32_t v[3] = {SENTINEL, SENTINEL, SENTINEL};
    const int ios = list_read_ints(buf.data(), MaxLineLength, v, 3);
    std::printf("%s|%s|%d|%d|%d|%d\n", tag, token.c_str(), ios,
                (int)v[0], (int)v[1], (int)v[2]);
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
    // inside the value rather than in padding. The repeat-count ceiling is
    // here at both sides of the boundary because `b93bad70` sits on it and
    // `evidence/ParseInAry_Opt/survivor_replay.txt` MEASURED that boundary
    // against gfortran ('199999999*7' and '200000000*7' give 7 7,
    // '200000001*7' gives 5010).
    const int L = MaxLineLength;
    const std::pair<const char*, std::string> LONG[] = {
        {"tail:digits", rep('0', L - 1) + "1"},
        {"tail:frac", "1." + rep('0', L - 3) + "5"},
        {"tail:word", rep('0', L - 4) + ",inf"},
        {"tail:wordUC", rep('0', L - 4) + ",nAn"},
        {"tail:star", rep('0', L - 2) + "2*"},
        {"tail:starval", rep('0', L - 4) + ",2*7"},
        {"tail:comma", rep('0', L - 2) + ",7"},
        {"tail:semi", rep('0', L - 3) + " ;7"},
        {"tail:sign", rep('0', L - 2) + ",+"},
        {"head:repeat", "2*7 3"},
        {"head:repeatnull", "3* 8"},
        {"head:semi", "1 ;2 3"},
        {"head:repmax", "200000000*7"},
        {"head:repover", "200000001*7"},
        {"head:repunder", "199999999*7"},
        {"head:ovf", "002147483648 7"},
        {"head:ovfmax", "2147483647 7"},
        {"head:slash", "1 / 2"},
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
