// VIT Translation Scaffold
// Function: ParseInput_Dbl_Opt
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE ParseInput_Dbl_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: f5ecce60e46e
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-20T00:02:34Z

#include "vit_types.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// Constants the reference names, read off the source rather than recalled.
// ---------------------------------------------------------------------------

// SysGnuLinux.f90:37 -- `CHARACTER(*), PARAMETER :: NewLine = ACHAR(10)`.
// Copied from parsedbary_opt.cpp (P4), same module, same argument.
constexpr char NewLine = '\n';

// Constants.f90:37 -- `CHARACTER(1), PARAMETER :: Tab = CHAR(9)`.
constexpr char Tab = '\t';

// ROSCO_Helpers.f90:44 -- `INTEGER(IntKi), PARAMETER :: MaxLineLength = 2048`.
// `CHARACTER(MaxLineLength) :: Line` is the buffer FindLine writes into, and it
// is also the length TRIM(Line) is taken over in the parse-error message.
constexpr int MaxLineLength = 2048;

// ROSCO_Helpers.f90:45 -- `INTEGER(IntKi), PARAMETER :: MaxParamLength = 200`.
// The element width of `CHARACTER(MaxParamLength) :: Words(2)`, which is what
// `GetWords`'s bridge takes as `len_Words` -- AND, for this unit, the RECORD
// LENGTH of the internal-file READ: the reference reads from `Words(1)`, not
// from `Line`. An internal file made from a scalar CHARACTER has exactly one
// record, of the variable's declared length, blank-padded. That is the one
// structural difference from the two `Parse*Ary_Opt` siblings, which read the
// 2048-byte `Line`.
constexpr int MaxParamLength = 200;

// `Int2LStr`'s result is `CHARACTER(11)` (ROSCO_Helpers.f90:1610). The bridge
// writes all 11 bytes, blank-padded, and does not NUL-terminate. Copied from
// parsedbary_opt.cpp (P4).
constexpr int Int2LStrLen = 11;

// `CHARACTER(*), PARAMETER :: RoutineName = 'ParseInput_Dbl_Opt'`.
constexpr std::string_view RoutineName = "ParseInput_Dbl_Opt";

// ROSCO_Helpers.f90:25 -- `LOGICAL, PARAMETER :: DEBUG_PARSING = .FALSE.`.
// A PARAMETER, so the debug block guarded by it is dead in every build of this
// tree; it is not translated and §2 of the evidence README says so. What IS
// translated is the `GetWords` call ABOVE the block, which the reference makes
// unconditionally.
constexpr bool DEBUG_PARSING = false;

// ---------------------------------------------------------------------------
// String helpers -- copied from parsedbary_opt.cpp (P4), same module.
// ---------------------------------------------------------------------------

// `TRIM(s)` -- drop TRAILING blanks, and blanks only. A NUL byte is not a
// blank and Fortran does not strip it; `strlen` would stop at one, which is
// why every length is carried explicitly.
std::string_view ftrim(const char* s, int len) {
    const std::string_view v(s, static_cast<std::size_t>(len));
    return v.substr(0, v.find_last_not_of(' ') + 1);
}

// `ErrVar%ErrMsg = <expr>` REALLOCATES to exactly LEN of the right-hand side.
// Copied byte for byte from parsedbary_opt.cpp, which copied it from
// chkparsedata.cpp and checkinputs.cpp (P4) -- including the `memset` to NUL
// past the new length, which unit #29 measured in both directions.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: ParseInput_Dbl_Opt: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: ParseInput_Dbl_Opt: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    std::memset(ErrVar->ErrMsg + s.size(), 0,
                static_cast<std::size_t>(ErrVar->n_ErrMsg_cap) - s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// `TRIM(Int2LStr(Num))`, through the integrated callee's own bridge. The result
// is CHARACTER(11), blank-padded, so the TRIM is the campaign's `ftrim` over
// exactly 11 bytes and not a `strlen`.
std::string int2lstr_trimmed(int Num) {
    char buf[Int2LStrLen];
    int2lstr_c(Num, buf);
    return std::string(ftrim(buf, Int2LStrLen));
}

// ---------------------------------------------------------------------------
// `READ (Words(1),*,IOSTAT=ErrStatLcl) Variable`
//
// LIST-DIRECTED INPUT OF ONE REAL(8) FROM AN INTERNAL FILE. The whole parser
// below -- `is_separator`, `is_blank`, `is_digit`, `lower`, `match_word`,
// `parse_real` and `list_read_reals` -- is COPIED FROM
// `translations/ROSCO_Helpers/parsedbary_opt.cpp` (P4), which measured every
// one of its rules against gfortran's own runtime in
// `evidence/ParseDbAry_Opt/fortran_io_probe{,2}.{f90,txt}`. Nothing about the
// item TYPE differs here: this unit reads REAL(DbKi) exactly as unit #54 does.
//
// TWO THINGS THAT **DO** DIFFER, and both are the caller's rather than the
// parser's:
//
//   * the record is `Words(1)`, 200 bytes, not `Line`, 2048;
//   * `n` is 1, not `SIZE(Ary)`.
//
// AND ONE CONSEQUENCE OF `n == 1` WORTH STATING BEFORE IT IS FOUND: `Variable`
// is `INTENT(INOUT)` and a SCALAR, so it is DEFINED on entry on every path.
// The three ways the reference leaves an item untransferred -- an END
// condition, a null value, a `/` terminator -- therefore leave `Variable`
// holding the caller's own value rather than undefined heap. That is why this
// unit needs no `no_oracle` entry where both `Parse*Ary_Opt` siblings needed
// one, and it is checked rather than asserted: §4 of the evidence README.
// ---------------------------------------------------------------------------

// libgfortran's `is_separator`, transcribed: the characters that legally END a
// list-directed value. ';' is in the set -- see the '1.0;2.0' row of unit #54's
// probe, which is the case that distinguishes "in the set" from "not in it".
bool is_separator(char c) {
    return c == '/' || c == ',' || c == '\n' || c == '\t' || c == ' '
           || c == '\r' || c == ';';
}

bool is_blank(char c) { return c == ' ' || c == '\t'; }

bool is_digit(char c) { return c >= '0' && c <= '9'; }

char lower(char c) {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + 32) : c;
}

// Does `rec[p..)` begin with `word`, case-insensitively?
bool match_word(const char* rec, int len, int p, std::string_view word) {
    if (p + static_cast<int>(word.size()) > len) {
        return false;
    }
    for (std::size_t k = 0; k < word.size(); ++k) {
        if (lower(rec[p + static_cast<int>(k)]) != word[k]) {
            return false;
        }
    }
    return true;
}

// WHAT `parse_real` RETURNS, and why two kinds of failure are not one.
// Unit #54's finding, copied with its measurement: libgfortran does not decide
// "is this a number" and then convert. It scans a field, and if it reaches a
// state where the field COULD have ended -- which a decimal point is enough to
// reach, because `1.` is a legal real -- it calls its conversion on what it
// has. `strtod` of a field with no digits yields 0.0, and THAT VALUE IS
// TRANSFERRED before the error is raised. Without the point the scanner rejects
// the field before converting anything and the item keeps the value it had.
enum class RealParse { Ok, BadNoStore, BadStoreZero };

// Parse ONE real starting at `p`. On `Ok` the value is written and `p` advances
// to the first character past the number. On either failure `p` is unspecified.
// The TERMINATOR is not checked here -- that is the caller's job, because a
// value terminated by a separator and one terminated by a stray character have
// different consequences (the ';' and '1.0.0' rows of the first probe).
RealParse parse_real(const char* rec, int len, int& p, double& out) {
    const int start = p;
    if (p < len && (rec[p] == '+' || rec[p] == '-')) {
        ++p;
    }

    // gfortran accepts the IEEE words, with the sign it has already consumed.
    // 'nan' may carry a parenthesised payload, which strtod also parses.
    if (match_word(rec, len, p, "infinity") || match_word(rec, len, p, "inf")
        || match_word(rec, len, p, "nan")) {   // NOLINT
        // THE WORD IS BOUNDED BY THE RECORD AND BY NOTHING ELSE. It used to be
        // copied into a `char buf[64]` under `n < 62`, which is the sibling's
        // code and is WRONG: `nan` may carry a parenthesised payload, and a
        // field of 63 characters or more was then handed to `strtod` with its
        // closing parenthesis cut off, so the conversion failed and the item
        // was left untouched. gfortran's own reader has no such bound.
        //
        // MEASURED BOTH WAYS before the repair, over every payload length a
        // 200-byte record can carry
        // (`evidence/ParseInput_Dbl_Opt/nan_payload_probe.{f90,cpp}`):
        //
        //   word length <=  62   both sides agree                    58 lengths
        //   word length  63..200 ref iostat=0 NaN, C++ iostat=5010  138 lengths
        //   word length     201  truncated by `Words(1)` itself; both fail
        //
        // The two surviving mutants of `62` -- `n <= 62` and `n < 63` -- were
        // mutants that made the translation LESS wrong, which is why no corpus
        // killed them. The same code is in `parsedbary_opt.cpp` and
        // `parseinary_opt.cpp`, where the record is 2048 bytes; escalated in
        // DECISIONS.md rather than edited from this unit.
        int q = p;
        while (q < len && !is_separator(rec[q])) {
            ++q;
        }
        char* end = nullptr;
        std::string word(1, rec[start] == '-' ? '-' : '+');
        word.append(rec + p, static_cast<std::size_t>(q - p));
        const double v = std::strtod(word.c_str(), &end);
        if (end == nullptr || *end != '\0') {
            return RealParse::BadNoStore;
        }
        out = v;
        p = q;
        return RealParse::Ok;
    }

    int digits = 0;
    bool saw_point = false;
    while (p < len && is_digit(rec[p])) { ++p; ++digits; }
    if (p < len && rec[p] == '.') {
        ++p;
        saw_point = true;
        while (p < len && is_digit(rec[p])) { ++p; ++digits; }
    }
    if (digits == 0 && !saw_point) {
        return RealParse::BadNoStore;
    }

    // Exponent: a letter form (e/E/d/D/q/Q) or gfortran's letterless
    // `1.5+2`. Either way at least one digit must follow.
    int exp_start = p;
    bool has_exp = false;
    if (p < len) {
        const char c = lower(rec[p]);
        if (c == 'e' || c == 'd' || c == 'q') {
            ++p;
            has_exp = true;
        } else if (rec[p] == '+' || rec[p] == '-') {
            has_exp = true;
        }
    }
    if (has_exp) {
        if (p < len && (rec[p] == '+' || rec[p] == '-')) {
            ++p;
        }
        int edigits = 0;
        while (p < len && is_digit(rec[p])) { ++p; ++edigits; }
        if (edigits == 0) {
            // A malformed exponent rejects BEFORE the conversion, whether or not
            // a point was seen: '-.e ' is not stored.
            return RealParse::BadNoStore;
        }
    }

    // A field with a point and no digits reaches the conversion; the conversion
    // yields 0.0 and the transfer happens, and only then is the error raised.
    if (digits == 0) {
        out = 0.0;
        return RealParse::BadStoreZero;
    }

    // The conversion itself is `strtod`, which is what libgfortran calls, so
    // the rounding is the same library routine on the same digits. Only the
    // exponent SPELLING is edited, and both of Fortran's spellings need one:
    // `1.5d2`/`1.5q2` have a letter C does not know, and `1.5+2` has no letter
    // at all. Measured: all three forms are accepted by the reference and give
    // 1.5E+02 (`fortran_io_probe.txt`, rows d-exponent / q-exponent /
    // bare-exponent).
    std::string text(rec + start, static_cast<std::size_t>(exp_start - start));
    if (has_exp) {
        text += 'e';
        std::string exp(rec + exp_start, static_cast<std::size_t>(p - exp_start));
        if (!exp.empty() && (lower(exp[0]) == 'e' || lower(exp[0]) == 'd'
                             || lower(exp[0]) == 'q')) {
            exp.erase(0, 1);
        }
        text += exp;
    }
    char* end = nullptr;
    out = std::strtod(text.c_str(), &end);
    return (end != nullptr && *end == '\0') ? RealParse::Ok
                                            : RealParse::BadNoStore;
}

// libgfortran's repeat-count ceiling, MEASURED at the boundary rather than
// recalled from its source, and measured HERE for a REAL item rather than
// carried over: '199999999*7' and '200000000*7' both give 7, '200000001*7'
// gives 5010. Same constant as the sibling's, which is the finding.
constexpr long MaxRepeat = 200000000;

// The transfer itself. `v` holds the items on entry and every item this returns
// without assigning keeps the value it arrived with -- which is the reference's
// behaviour and is what makes `Variable` an output on the failure path too.
int list_read_reals(const char* rec, int len, double* v, int n) {
    int p = 0;
    int i = 0;
    while (i < n) {
        while (p < len && is_blank(rec[p])) { ++p; }
        if (p >= len) {
            return -1;                       // END: the record ran out
        }
        if (rec[p] == ',') {                 // a null value: item unchanged
            ++p;
            ++i;
            continue;
        }
        if (rec[p] == '/') {                 // terminate; the rest is unchanged
            return 0;
        }
        if (rec[p] == ';') {
            return 5010;
        }

        // An optional repeat count `r*`, r a positive integer no greater than
        // `MaxRepeat`. `02*7` is 7 7, so leading zeros are part of the count.
        //
        // THE CEILING WAS MISSING HERE and the sibling has it. `std::strtol`
        // SATURATES at `LONG_MAX` on overflow, so `999999999999999999999*7`
        // came back as a positive repeat and the item was stored; the reference
        // rejects it. Measured at the boundary for a REAL item, which is the
        // half unit #55 could not answer -- it measured an INTEGER one --
        // `evidence/ParseInput_Dbl_Opt/record_form_probe.txt`:
        //
        //   199999999*7   ref 0     got 0      the rung below
        //   200000000*7   ref 0     got 0      the ceiling itself
        //   200000001*7   ref 5010  got 0      <- the divergence
        //   999999...9*7  ref 5010  got 0      <- and its wide form
        //
        // A repeat count is record GRAMMAR rather than item TYPE, so the
        // sibling's constant carries and the three rows above are the check
        // rather than the assumption (RUNBOOK: item type against record
        // grammar). The accumulation is copied from
        // `translations/ROSCO_Helpers/parseinary_opt.cpp` (P4) so the overflow
        // is detected during the accumulation rather than after it.
        long repeat = 1;
        int q = p;
        while (q < len && is_digit(rec[q])) { ++q; }
        if (q > p && q < len && rec[q] == '*') {
            long count = 0;
            bool count_over = false;
            for (int k = p; k < q; ++k) {
                count = count * 10 + (rec[k] - '0');
                if (count > MaxRepeat) {
                    count_over = true;
                    count = MaxRepeat + 1;
                }
            }
            if (count <= 0 || count_over || count > MaxRepeat) {
                return 5010;                    // '0*1 2' and '200000001*7'
            }
            repeat = count;
            p = q + 1;
            // `r*` with nothing after it is r NULL values.
            if (p >= len || is_separator(rec[p])) {
                for (long k = 0; k < repeat && i < n; ++k) {
                    ++i;
                }
                while (p < len && is_blank(rec[p])) { ++p; }
                if (p < len && rec[p] == ',') { ++p; }
                continue;
            }
        }

        double value = 0.0;
        const RealParse verdict = parse_real(rec, len, p, value);
        if (verdict == RealParse::BadNoStore) {
            return 5010;
        }
        if (verdict == RealParse::BadStoreZero) {
            // The point-without-digits field: converted, transferred, then
            // failed. The terminator still decides -- '..' has a point and is
            // NOT stored, because the second point is a stray character rather
            // than a separator.
            if (p < len && !is_separator(rec[p])) {
                return 5010;
            }
            v[i] = value;
            return 5010;
        }
        if (p < len && !is_separator(rec[p])) {
            return 5010;
        }
        for (long k = 0; k < repeat && i < n; ++k) {
            v[i++] = value;
        }
        while (p < len && is_blank(rec[p])) { ++p; }
        if (p < len && rec[p] == ',') { ++p; }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// gfortran's LIST-DIRECTED OUTPUT, for the one record this unit writes.
//
// `field`, `nonfinite_text` and `list_directed_real` are copied from
// `parsedbary_opt.cpp` (P4), which copied them from
// `powercontrolsetpoints.cpp`, where they are measured against 22,526 records
// written by the reference's own runtime
// (`evidence/PowerControlSetpoints/ld_probe.txt`). Nothing is re-derived.
//
// THE RECORD THIS UNIT WRITES, ROSCO_Helpers.f90:243 (clean):
//
//   PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//
//            " in input file.  Using default value of ", Variable
//
// Two items: a CHARACTER expression and a REAL(8). Unit #54's measured layout
// rules give the record directly -- one leading blank starts it, a CHARACTER
// item is written raw, there is NO separator between a CHARACTER and a
// following real, and a REAL is a self-contained 26-byte field. The record is
// therefore ' ' + <text> + list_directed_real(Variable), and it is COMPARED:
// `harness/ranges.toml` carries `vit_record = { compare_record = ... }` for
// this unit, so the reference's own record is the oracle on every case that
// reaches this arm.
//
// AND ONE FACT ABOUT WHAT CAN REACH IT. `Variable = 0` is the statement
// IMMEDIATELY ABOVE the PRINT and is unconditional, so the only value this
// statement can ever write is +0.0 -- which takes the `decexp` in [-1, 16]
// branch of `list_directed_real` with d = 16 and never the E-form. That is a
// property of the PROGRAM, not of the corpus, and it is why the E-form's lines
// are declared `unreachable` in the mutation sweep from measured line coverage
// rather than argued about. §7 of the evidence README.
// ---------------------------------------------------------------------------

// Right-justify `text` in a field of `w`, or fill with '*' on overflow.
std::string field(const std::string& text, int w) {
    if (static_cast<int>(text.size()) > w) {
        return std::string(static_cast<std::size_t>(w), '*');
    }
    return std::string(static_cast<std::size_t>(w) - text.size(), ' ') + text;
}

// gfortran renders a non-finite value as a WORD. Copied from
// powercontrolsetpoints.cpp (P4).
bool nonfinite_text(double v, std::string& out) {
    if (std::isnan(v)) { out = "NaN"; return true; }
    if (std::isinf(v)) {
        out = std::signbit(v) ? "-Infinity" : "Infinity";
        return true;
    }
    return false;
}

// One list-directed field for a REAL(8).
std::string list_directed_real(double v) {
    std::string text;
    if (nonfinite_text(v, text)) {
        return field(text, 26);
    }

    char tmp[512];
    std::snprintf(tmp, sizeof tmp, "%.16E", v);
    std::string s(tmp);
    const std::size_t epos = s.find('E');
    const int decexp = std::atoi(s.c_str() + epos + 1);

    if (decexp >= -1 && decexp <= 16) {
        const int d = 16 - decexp;
        std::snprintf(tmp, sizeof tmp, "%.*f", d, v);
        text = tmp;
        if (d == 0) {
            text += '.';
        }
        return field(text, 21) + std::string(5, ' ');
    }

    const std::string mant = s.substr(0, epos);
    const char esign = s[epos + 1];
    std::string edig = s.substr(epos + 2);
    while (edig.size() > 1 && edig[0] == '0') {
        edig.erase(0, 1);
    }
    if (edig.size() > 3) {
        return field(std::string(27, '*'), 26);
    }
    edig.insert(0, 3 - edig.size(), '0');
    return field(mant + "E" + esign + edig, 26);
}

// `PRINT *, "ROSCO Warning: Did not find "//TRIM(VarName)//" in input file.  "
//           //"Using default value of ", Variable` on unit 6.
void print_default_warning(std::string_view VarName, double Variable) {
    std::string rec(1, ' ');
    rec += "ROSCO Warning: Did not find ";
    rec.append(VarName);
    rec += " in input file.  Using default value of ";
    // NO separator between a CHARACTER item and a REAL that follows it -- unit
    // #54's measured rule. The 26-byte field carries its own leading blanks.
    rec += list_directed_real(Variable);
    rec += '\n';
    std::fwrite(rec.data(), 1, rec.size(), stdout);
}

}  // namespace

void ParseInput_Dbl_Opt(char* FileLines, int n_FileLines, int len_FileLines,
                        char* VarName, int len_VarName,
                        double* Variable,
                        char* FileName, int len_FileName,
                        errorvariables_view_t* ErrVar,
                        int has_AllowDefault, int32_t AllowDefault,
                        int has_UnEc, int UnEc) {
    // NO-OP STUB -- the whole body deleted. The red test for the differential
    // harness itself: whatever this moves is what the harness can see at all.
    //
    // THE THREE CALLEE CALLS ARE KEPT BEHIND A GUARD NO CASE SATISFIES, and
    // they are not dead code to be tidied away. `vit/test_validate.generate_
    // callee_bridges` decides whether to emit `<stem>_callees.o` by running a
    // regex over THIS FILE's text; a stub that calls nothing links against a
    // bridge that was never generated, and the failure is a link error in
    // OTHER integrated units (unit #45). `n_FileLines` is an extent and is
    // never negative.
    if (n_FileLines < -1) {
        int32_t f = 0; int ln = 0;
        char one = 0;
        findline_c(FileLines, n_FileLines, len_FileLines, VarName, len_VarName,
                   &f, &one, &ln, 0, 0);
        getwords_c(&one, 1, &one, 1, 1);
        char b[Int2LStrLen];
        int2lstr_c(0, b);
        (void)b;
    }
    (void)Variable; (void)FileName; (void)len_FileName; (void)ErrVar;
    (void)has_AllowDefault; (void)AllowDefault; (void)has_UnEc; (void)UnEc;
    (void)ftrim; (void)assign_errmsg; (void)int2lstr_trimmed;
    (void)list_read_reals; (void)print_default_warning;
    (void)DEBUG_PARSING; (void)NewLine; (void)Tab; (void)MaxLineLength;
    (void)MaxParamLength; (void)RoutineName;
}
