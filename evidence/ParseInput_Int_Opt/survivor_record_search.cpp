// SEARCH FOR A RECORD THAT DISTINGUISHES A SURVIVING MUTANT FROM THE SHIPPED
// TRANSLATION -- the question the mutation score asks, taken off the corpus and
// asked directly of the two programs.
//
// COPIED IN SHAPE from `evidence/ParseInput_Dbl_Opt/survivor_record_search.cpp`
// (P4) -- the argument, the abort-stub callees, the textual include, the
// 400-byte object, the overridable `Words(2)` and the negative control are all
// that file's. TWO THINGS ARE THIS UNIT'S OWN:
//
//   * the reader is `list_read_ints`, and the item is an `int32_t` printed as a
//     decimal rather than a `double` printed as bits;
//   * THE SPACE IS SPLIT INTO ADMISSIBLE AND FUNCTION-ONLY RECORDS, and that is
//     the correction rather than a decoration -- see below.
//
// WHY. A surviving mutant is one of three things: equivalent, unreached by the
// corpus, or a blind spot. The mutation sweep cannot tell them apart, because
// its answer for all three is the same word. `run_survivor_record_search.sh`
// builds this file once against the SHIPPED translation and once against each
// surviving mutant, runs both over the same record space, and diffs.
//
// ADMISSIBLE VERSUS FUNCTION-ONLY, AND WHY IT IS IN THE TAG RATHER THAN IN THE
// PROSE. Unit #56 made the same mistake twice in this instrument and recorded
// both: it read a `differs` as a corpus lever for records that `GetWords` can
// never produce. `GetWords` (ROSCO_Helpers.f90) splits on
//
//     NextWhite = SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )
//     Words(IW) = Line(Ch+1:Ch+NextWhite-1)
//
// so a word contains NONE of space, comma, '!', semicolon, apostrophe, quote or
// tab, and it is LEFT-JUSTIFIED into a blank-padded 200-byte element. Therefore
// a record admissible to THE UNIT is exactly `<word><blanks>`:
//
//   * no interior blank, no comma, no semicolon anywhere;
//   * nothing right-justified -- a TAIL record is a record only the FUNCTION
//     can be handed.
//
// Records of both kinds are searched, because a difference on a function-only
// record is still a fact about the two programs and is worth reporting -- but
// the tag says which, so nobody reads the second as a corpus lever.
//
// WHY THE BUFFER IS 400 BYTES, AND WHY THAT IS FIDELITY RATHER THAN PADDING.
// In the reference, `Words` is `CHARACTER(MaxParamLength) :: Words(2)` -- one
// contiguous 400-byte object -- and the READ's internal unit is `Words(1)`, its
// first 200 bytes. Byte 201 of that object IS `Words(2)(1:1)`, the FIRST
// CHARACTER OF THE PARAMETER NAME `FindLine` just matched, and the translation
// lays the two out the same way (`std::vector<char> Words(2 * MaxParamLength)`).
// So a mutant that reads `rec[200]` is not reading past an allocation; it is
// reading a letter, and `--sanitize` is correct to say nothing about it. The
// SECOND HALF OF THE BUFFER is what turns that class into a VALUE difference.
//
// IT IS NOT A DECLARATION AND IT IS NOT FOLDED INTO THE SCORE. It compares the
// translation against ITSELF-MUTATED, so it says nothing about whether either
// agrees with the Fortran reference; that is the differential harness's job.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -fsanitize=address,undefined \
//              -I<dir with vit_types.h> -DVIT_TRANSLATION='"<path to a .cpp>"' \
//              survivor_record_search.cpp -o search
//   run:   ./search            > one line per record

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int);
void getwords_c(char*, int, char*, int, int);
void int2lstr_c(int, char*);

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
// (`5002a20d`, `200` -> `201`), and a search that read it would move its own
// record space with the mutant instead of asking the two programs the same
// question. That also makes `5002a20d` the NEGATIVE CONTROL: it is one open
// survivor this search cannot reach, and it must come back `NONE`.
const int REC = 200;    // the record: Words(1)
const int BUF = 400;    // the whole object: Words(1) and Words(2)

// The ADMISSIBLE alphabet: every character the parser's own predicates test,
// minus the seven `GetWords` splits on. `x` is in none of the predicates, so a
// "matches nothing" case exists.
//
// CR AND LF ARE NOT IN IT, ON PURPOSE, AND THEY ARE NOT MISSING EITHER. Both
// are admissible characters -- neither is a `GetWords` separator -- but adding
// them here would renumber the whole enumeration and move the FIRST-differing
// record of rows that `mutation/*.equivalences.md` quotes. They are searched in
// the EOLADM block at the end of `main`, which is appended rather than
// inserted, so every existing row is byte-identical (P5).
const char ALPHABET[] = "0123456789+-.eEdDqQnNaAiIfFtTyYxX/*";

// The characters a word can NEVER contain, searched separately and tagged
// FUNCONLY so a difference on one is not read as a corpus lever.
const char FORBIDDEN[] = " ,;";

// What `GetWords` leaves in `Words(2)`: the parameter name `FindLine` matched,
// left-justified and blank-padded. The DEFAULT is a real name from
// `Examples/DISCON.IN`, and `survivor_record_search.txt` is the run at that
// default.
//
// IT IS OVERRIDABLE BECAUSE IT IS A PARAMETER OF THE QUESTION, not a constant
// of the program -- unit #56's finding, which cost that unit five wrong
// equivalences before it was made. `Words(2)` is whatever `VarName` was, and in
// this unit's corpus `VarName` is an arbitrary CHARACTER(*): it can begin with
// a digit, a sign or a star, none of which a Fortran parameter name does.
#ifndef VIT_WORDS2
#define VIT_WORDS2 "PC_KP"
#endif
const char WORDS2[] = VIT_WORDS2;

// The sentinel is what an item the transfer never reached looks like. The
// INTEGER reader stores nothing on any failure -- measured over the fourteen
// 5010 rows of `record_form_probe.txt` -- so an untouched slot is the normal
// outcome of a rejection and must be distinguishable from a stored 0.
const std::int32_t SENTINEL = -987654;

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
    std::int32_t v = SENTINEL;
    const int ios = list_read_ints(buf.data(), REC, &v, 1);
    std::printf("%s|%s|%d|%d\n", tag, token.c_str(), ios, static_cast<int>(v));
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

// A full-width record: `body` left-justified and padded with `pad` to exactly
// 200 characters, so the record ends INSIDE the value and a scan that runs one
// past it lands on `Words(2)`.
std::string fullw(const std::string& body, char pad = '0') {
    if (body.size() >= static_cast<std::size_t>(REC)) return body.substr(0, REC);
    return rep(pad, REC - static_cast<int>(body.size())) + body;
}

}  // namespace

int main() {
    const std::size_t A = std::strlen(ALPHABET);

    // ---- ADMISSIBLE: a word left-justified, blanks after ------------------
    for (std::size_t i = 0; i < A; ++i) {
        const std::string t(1, ALPHABET[i]);
        report("HEAD", t, head(t));
    }
    for (std::size_t i = 0; i < A; ++i)
        for (std::size_t j = 0; j < A; ++j) {
            std::string t;
            t += ALPHABET[i];
            t += ALPHABET[j];
            report("HEAD", t, head(t));
        }
    for (std::size_t i = 0; i < A; ++i)
        for (std::size_t j = 0; j < A; ++j)
            for (std::size_t k = 0; k < A; ++k) {
                std::string t;
                t += ALPHABET[i];
                t += ALPHABET[j];
                t += ALPHABET[k];
                report("HEAD", t, head(t));
            }

    // ---- ADMISSIBLE: full-width words, 200 bytes exactly ------------------
    //
    // Leading zeros are what make a full-width INTEGER record legal: they are
    // part of the value (`0000...042` is 42) or part of a repeat count
    // (`0000...03*7` is 7), so the significant digits sit at the record's END
    // and a scan reaches the last byte with the answer still depending on it.
    // A 200-digit run cannot do that -- it overflows -- so it is here as the
    // rejection at the boundary rather than as an acceptance.
    const std::pair<const char*, std::string> FULLADM[] = {
        {"full:digits", rep('0', REC - 1) + "1"},
        {"full:intmax", fullw("2147483647")},
        {"full:intmin", fullw("-2147483648")},
        {"full:ovfmax", fullw("2147483648")},
        {"full:ovfmin", fullw("-2147483649")},
        {"full:allnine", rep('9', REC)},
        {"full:rep", fullw("3*7")},
        {"full:repnull", fullw("3*")},
        {"full:repone", fullw("1*7")},
        {"full:repzero", fullw("0*7")},
        {"full:repceil", fullw("200000000*7")},
        {"full:repover", fullw("200000001*7")},
        {"full:star", rep('0', REC - 1) + "*"},
        {"full:starsign", fullw("3*-")},
        {"full:starplus", fullw("3*+")},
        {"full:slash", fullw("7/")},
        {"full:slashpre", fullw("/7")},
        {"full:signonly", fullw("-")},
        {"full:point", fullw("7.")},
        {"full:word", fullw("nan")},
        {"full:letter", fullw("7x")},
        {"full:trailrep", rep('0', REC - 4) + "03*7"},
        {"full:repdig", fullw("2*7")},
    };
    for (const auto& r : FULLADM) {
        std::string rec = r.second;
        if (rec.size() > static_cast<std::size_t>(REC)) {
            std::fprintf(stderr, "survivor_record_search: %s is %zu characters, "
                                 "longer than the %d-byte record\n",
                         r.first, rec.size(), REC);
            return 2;
        }
        rec.resize(static_cast<std::size_t>(REC), ' ');
        report("FULLADM", r.first, rec);
    }

    // ---- ADMISSIBLE: short words that reach a named branch ----------------
    const std::pair<const char*, const char*> WORDADM[] = {
        {"w:plain", "42"},      {"w:signed", "+42"},   {"w:neg", "-42"},
        {"w:zeros", "000042"},  {"w:imax", "2147483647"},
        {"w:imin", "-2147483648"}, {"w:ovf", "2147483648"},
        {"w:rep", "3*7"},       {"w:repnull", "3*"},   {"w:repzero", "0*7"},
        {"w:repone", "1*7"},    {"w:repceil", "200000000*7"},
        {"w:repover", "200000001*7"}, {"w:repwide", "999999999999999999999*7"},
        {"w:slash", "/"},       {"w:slashaft", "7/9"}, {"w:slashpre", "/7"},
        {"w:dot", "."},         {"w:realpt", "1.5"},   {"w:exp", "1e2"},
        {"w:sign", "-"},        {"w:plus", "+"},       {"w:star", "*"},
        {"w:starval", "*7"},    {"w:nan", "nan"},      {"w:inf", "inf"},
        {"w:trail", "5abc"},    {"w:repslash", "3*/"}, {"w:repstar", "3**"},
    };
    for (const auto& r : WORDADM) report("WORDADM", r.first, head(r.second));

    // ---- FUNCTION-ONLY: records `GetWords` cannot produce -----------------
    //
    // Right-justified tokens, and tokens containing a blank, a comma or a
    // semicolon. A difference here is a fact about `list_read_ints` and NOT a
    // corpus lever, because no input to `ParseInput_Int_Opt` can put such a
    // record in `Words(1)`. Searched anyway: an equivalence claim that rests on
    // the caller needs to say what the FUNCTION does differently, or it is
    // indistinguishable from a claim that the function agrees with itself.
    const std::size_t F = std::strlen(FORBIDDEN);
    for (std::size_t i = 0; i < A; ++i) {
        const std::string t(1, ALPHABET[i]);
        report("FUNCONLY:tail", t, tail(t));
    }
    for (std::size_t f = 0; f < F; ++f)
        for (std::size_t i = 0; i < A; ++i)
            for (std::size_t j = 0; j < A; ++j) {
                std::string t;
                t += ALPHABET[i];
                t += FORBIDDEN[f];
                t += ALPHABET[j];
                report("FUNCONLY:sep", t, head(t));
                report("FUNCONLY:septail", t, tail(t));
            }
    const std::pair<const char*, std::string> FULLFUN[] = {
        {"full:comma", rep('0', REC - 2) + ",7"},
        {"full:semi", rep('0', REC - 3) + " ;7"},
        {"full:blank", rep('0', REC - 2) + " 7"},
        {"full:commaend", rep('0', REC - 1) + ","},
        {"full:semiend", rep('0', REC - 1) + ";"},
        {"full:blankend", rep('0', REC - 1) + " "},
        {"full:repcomma", fullw("3*,")},
        {"full:repblank", fullw("3* ")},
        {"full:repsemi", fullw("3*;")},
        {"head:nullfirst", ",7"},
        {"head:semifirst", ";7"},
        {"head:semiafter", "1 ;2"},
        {"head:semitwice", "1 ;;2"},
        {"head:commasemi", "1, ;2"},
        {"head:slashsp", "1 / 2"},
        {"head:repsp", "3* 8"},
        {"head:twoval", "1 2"},
        {"head:commaval", "1,2"},
    };
    for (const auto& r : FULLFUN) {
        std::string rec = r.second;
        rec.resize(static_cast<std::size_t>(REC), ' ');
        report("FUNCONLY", r.first, rec);
    }

    // ---- ADMISSIBLE: records carrying an END-OF-LINE byte -----------------
    //
    // ADDED AT THE SECOND DISPATCH, AND APPENDED LAST RATHER THAN GROUPED WITH
    // THE OTHER ADMISSIBLE BLOCKS. That is P5, not untidiness: the runner
    // reports the FIRST record on which a mutant differs, so inserting records
    // anywhere earlier would move rows that `mutation/*.equivalences.md`
    // already quotes. Appended, every existing row is byte-identical and only a
    // mutant that NOTHING else distinguishes gets a row from here.
    //
    // WHY IT EXISTS. The first dispatch left two survivors open and said of
    // them: "Neither is a NONE this search failed to explain -- the search's own
    // alphabet has no CR or LF either." True, and it made every one of the 37
    // declared equivalences a claim over a space that could not contain the one
    // byte the two survivors turned on. The corpus has since gained CR and LF
    // records and killed both survivors; this block asks the same question of
    // the OTHER 52, over a space three thousand records wider rather than over
    // the handful of cases the corpus carries.
    //
    // EVERY RECORD HERE IS ADMISSIBLE. Neither byte is in `GetWords`' separator
    // set (`' ,!;''"'//Tab`, ROSCO_Helpers.f90:145), so a run containing one is
    // copied into `Words(1)` whole -- which is what makes this an ADMISSIBLE
    // block and not a FUNCONLY one, whatever its position in the file.
    //
    // THE LABEL IS PRINTABLE AND THE RECORD IS NOT. The token is spelled `cr`
    // or `lf` where the record holds the byte, so the row format is unchanged
    // and no reader has to un-escape anything.
    {
        const char EOLB[2] = {'\r', '\n'};
        const char* EOLN[2] = {"cr", "lf"};
        for (int e = 0; e < 2; ++e) {
            const char c = EOLB[e];
            const std::string n = EOLN[e];
            // one alphabet character, the end-of-line before it and after it
            for (std::size_t i = 0; i < A; ++i) {
                const std::string t(1, ALPHABET[i]);
                report("EOLADM", n + "+" + t, head(std::string(1, c) + t));
                report("EOLADM", t + "+" + n, head(t + std::string(1, c)));
            }
            // two alphabet characters, the end-of-line in each of three places
            for (std::size_t i = 0; i < A; ++i)
                for (std::size_t j = 0; j < A; ++j) {
                    std::string t;
                    t += ALPHABET[i];
                    t += ALPHABET[j];
                    report("EOLADM", n + "+" + t,
                           head(std::string(1, c) + t));
                    report("EOLADM", t.substr(0, 1) + "+" + n + "+" + t.substr(1),
                           head(t.substr(0, 1) + std::string(1, c) + t.substr(1)));
                    report("EOLADM", t + "+" + n,
                           head(t + std::string(1, c)));
                }
            // the record that is ALL end-of-line, at the width that makes the
            // LEADING scan reach the record's last byte -- the one shape no
            // other block here can produce, because a record of all blanks is
            // not a word
            report("EOLADM", "full:all" + n, rep(c, REC));
            report("EOLADM", "full:" + n + "then1", rep(c, REC - 1) + "1");
            report("EOLADM", "full:1then" + n, "1" + rep(c, REC - 1));
            report("EOLADM", "full:" + n + "slash", rep(c, REC - 1) + "/");
            report("EOLADM", "full:" + n + "star", rep(c, REC - 1) + "*");
            report("EOLADM", "full:" + n + "val", fullw("3*7") .substr(0, REC - 1) + std::string(1, c));
            report("EOLADM", "head:" + n + "only", head(std::string(1, c)));
            report("EOLADM", "head:" + n + "slash", head(std::string(1, c) + "/"));
            report("EOLADM", "head:" + n + "rep", head(std::string(1, c) + "3*7"));
            report("EOLADM", "head:rep" + n, head("3*" + std::string(1, c)));
            report("EOLADM", "head:" + n + "sign", head(std::string(1, c) + "-7"));
            report("EOLADM", "head:val" + n + "val", head("7" + std::string(1, c) + "8"));
        }
        // the two bytes TOGETHER, which is what a DOS line ending would be and
        // is the one shape a single-byte loop cannot make
        report("EOLADM", "full:crlf", rep('\r', REC / 2) + rep('\n', REC - REC / 2));
        report("EOLADM", "head:crlf", head("\r\n7"));
        report("EOLADM", "head:lfcr", head("\n\r7"));
        report("EOLADM", "head:7crlf", head("7\r\n"));
    }
    return 0;
}
