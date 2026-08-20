// SEARCH FOR A RECORD THAT DISTINGUISHES A SURVIVING MUTANT FROM THE SHIPPED
// TRANSLATION -- the question the mutation score asks, taken off the corpus and
// asked directly of the two programs.
//
// COPIED IN SHAPE from `evidence/ParseInAry_Opt/survivor_record_search.cpp`
// (P4) -- the argument below, the alphabet, the HEAD/TAIL enumeration, the
// abort-stub callees, the textual include and the negative control are all that
// file's. THREE THINGS ARE THIS UNIT'S OWN, and each of them is the reason the
// sibling's file could not simply be run here:
//
//   * the reader is `list_read_reals` with `n == 1`, because the reference
//     transfers ONE item;
//   * the record is 200 bytes -- `Words(1)`, a `CHARACTER(MaxParamLength)` --
//     not `Line`'s 2048;
//   * THE BUFFER IS 400 BYTES AND THE SECOND HALF IS NOT BLANK. That is the
//     whole point of this file and it is explained below.
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
// WHY THE BUFFER IS 400 BYTES, AND WHY THAT IS FIDELITY RATHER THAN PADDING.
// In the reference, `Words` is `CHARACTER(MaxParamLength) :: Words(2)` -- one
// contiguous 400-byte object -- and the READ's internal unit is `Words(1)`,
// its first 200 bytes. Byte 201 of that object IS `Words(2)(1:1)`, the FIRST
// CHARACTER OF THE PARAMETER NAME `FindLine` just matched, and the translation
// lays the two out the same way (`std::vector<char> Words(2 * MaxParamLength)`).
//
// So a mutant that reads `rec[200]` -- and eleven of this unit's twenty-three
// open survivors are `rec[p]` -> `rec[p + 1]` or `p < len` -> `p <= len` -- is
// NOT reading past an allocation. It is reading a letter. `vit_mutate.py
// --sanitize` therefore cannot see this class here, which is exactly the
// opposite of the sibling, where `rec` is a 2048-byte vector and byte 2049 is
// past its end. The sanitiser is still switched on in the build (the runner
// passes it) as a guard on the search itself, but it is not what does the work:
// the SECOND HALF OF THE BUFFER IS, because it makes the difference a VALUE
// difference instead of an address one.
//
// `WORDS2` below is a plausible parameter name, left-justified and
// blank-padded, which is what `GetWords` writes into `Words(2)`. Its first
// character is a letter on purpose: a blank there would make the two programs
// agree again and the search would report `none` for the class it exists to
// reach.
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
//          START of the 200-byte record with blanks after.
//   TAIL   the same strings at the END, so the last character of the token is
//          the last byte of the record and a scan that runs one past it lands
//          on `Words(2)`.
//   FULL   crafted records that fill all 200 bytes -- the shape in which a
//          separator scan reaches the record's last byte instead of stopping
//          in blank padding -- plus the boundaries the parser's own predicates
//          name.
//
// Textual include, because the parser lives in an anonymous namespace and a
// copy here would go stale silently.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -fsanitize=address,undefined \
//              -I<test dir> -DVIT_TRANSLATION='"<path to a .cpp>"' \
//              survivor_record_search.cpp -o search
//   run:   ./search            > one line per record

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "parseinput_dbl_opt_callees.h"

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

// LITERALS, NOT `MaxParamLength`. The constant is itself a mutation site
// (`479d0b11`, `200` -> `201`), and a search that read it would move its own
// record space with the mutant instead of asking the two programs the same
// question. That also makes `479d0b11` the NEGATIVE CONTROL: it is the one open
// survivor this search cannot reach, and it must come back `NONE`.
const int REC = 200;    // the record: Words(1)
const int BUF = 400;    // the whole object: Words(1) and Words(2)

// The alphabet is every character the parser's own predicates test, plus the
// letters of `inf` / `infinity` / `nan` and their uppercase forms, plus one
// character (`x`) that is in none of them so a "matches nothing" case exists.
// Kept character for character from the sibling's so the two spaces are
// comparable.
const char ALPHABET[] = "0123456789+-.eEdDqQnNaAiIfFtTyYxX ,;/*()";

// What `GetWords` leaves in `Words(2)`: the parameter name `FindLine` matched,
// left-justified and blank-padded. A real name from `Examples/DISCON.IN`.
const char WORDS2[] = "PC_KP";

// The sentinel is what an item the transfer never reached looks like, printed
// as a bit pattern so a stored `0.0` is distinguishable from an untouched slot.
const double SENTINEL = -987.654;

std::vector<char> object(const std::string& rec200) {
    std::vector<char> b(static_cast<std::size_t>(BUF), ' ');
    std::memcpy(b.data(), rec200.data(),
                rec200.size() < static_cast<std::size_t>(REC)
                    ? rec200.size() : static_cast<std::size_t>(REC));
    std::memcpy(b.data() + REC, WORDS2, std::strlen(WORDS2));
    return b;
}

void report(const char* tag, const std::string& token, const std::string& rec) {
    std::vector<char> buf = object(rec);
    double v = SENTINEL;
    const int ios = list_read_reals(buf.data(), REC, &v, 1);
    std::uint64_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    std::printf("%s|%s|%d|%016llx\n", tag, token.c_str(), ios,
                (unsigned long long)bits);
}

std::string head(const std::string& tok) {
    std::string r(static_cast<std::size_t>(REC), ' ');
    std::memcpy(&r[0], tok.data(), tok.size());
    return r;
}

std::string tail(const std::string& tok) {
    std::string r(static_cast<std::size_t>(REC), ' ');
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

    // The full-width records. Each is exactly 200 characters, so the record
    // ends inside the value rather than in blank padding and a scan that runs
    // one past the end lands on `Words(2)`.
    const int L = REC;
    const std::pair<const char*, std::string> FULL[] = {
        {"full:digits", rep('0', L - 1) + "1"},
        {"full:frac", "1." + rep('0', L - 3) + "5"},
        {"full:word", rep('0', L - 4) + ",inf"},
        {"full:wordUC", rep('0', L - 4) + ",nAn"},
        {"full:infpad", "inf" + rep(' ', L - 3)},
        {"full:inftail", rep(' ', L - 3) + "inf"},
        {"full:nantail", rep(' ', L - 3) + "nan"},
        {"full:infinity", rep(' ', L - 8) + "infinity"},
        {"full:star", rep('0', L - 2) + "2*"},
        {"full:starval", rep('0', L - 4) + ",2*7"},
        {"full:comma", rep('0', L - 2) + ",7"},
        {"full:semi", rep('0', L - 3) + " ;7"},
        {"full:sign", rep('0', L - 2) + ",+"},
        {"full:expE", rep('0', L - 3) + "1E1"},
        {"full:expD", rep('0', L - 3) + "1D1"},
        {"full:expQ", rep('0', L - 3) + "1Q1"},
        {"full:expbare", rep('0', L - 3) + "1+1"},
        {"full:expcut", rep('0', L - 2) + "1E"},
        {"full:pointcut", rep(' ', L - 1) + "."},
        {"full:pointexp", rep(' ', L - 3) + ".e1"},
        {"full:signcut", rep(' ', L - 1) + "-"},
        {"full:digitcut", rep(' ', L - 1) + "7"},
        {"full:longnan", "nan(" + rep('0', 70) + ")"},
        {"full:nan62", "nan(" + rep('0', 57) + ")"},
        {"full:nan63", "nan(" + rep('0', 58) + ")"},
        {"head:repeat", "2*7 3"},
        {"head:repeatnull", "3* 8"},
        {"head:semi", "1 ;2 3"},
        {"head:slash", "1 / 2"},
        {"head:null", ",7"},
        {"head:zerorep", "0*7"},
        {"head:negrep", "-1*7"},
    };
    for (const auto& r : FULL) {
        if (r.second.size() > static_cast<std::size_t>(L)) {
            std::fprintf(stderr, "survivor_record_search: %s is %zu characters, "
                                 "longer than the %d-byte record\n",
                         r.first, r.second.size(), L);
            return 2;
        }
        std::string rec = r.second;
        rec.resize(static_cast<std::size_t>(L), ' ');
        report("FULL", r.first, rec);
    }
    return 0;
}
