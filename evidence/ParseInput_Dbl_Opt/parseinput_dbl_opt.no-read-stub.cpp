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
        char buf[64];
        int n = 0;
        int q = p;
        while (q < len && n < 62 && !is_separator(rec[q])) {
            buf[n++] = rec[q++];
        }
        buf[n] = '\0';
        char* end = nullptr;
        std::string word(1, rec[start] == '-' ? '-' : '+');
        word += buf;
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

        // An optional repeat count `r*`, r a positive integer.
        long repeat = 1;
        int q = p;
        while (q < len && is_digit(rec[q])) { ++q; }
        if (q > p && q < len && rec[q] == '*') {
            repeat = std::strtol(std::string(rec + p, static_cast<std::size_t>(q - p)).c_str(),
                                 nullptr, 10);
            if (repeat <= 0) {
                return 5010;
            }
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
    // ! Figure out if we allow default
    // AllowDefault_ = .TRUE.
    // if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault
    bool AllowDefault_ = true;
    if (has_AllowDefault) {
        AllowDefault_ = (AllowDefault != 0);
    }

    // ! If we've already failed, don't read anything
    // IF (ErrVar%aviFAIL >= 0) THEN
    if (ErrVar->aviFAIL < 0) {
        return;
    }

    // CALL FindLine(FileLines, VarName, FoundLine, Line, CurLine)
    //
    // `AryLen` is OPTIONAL in `FindLine` and is NOT passed here, so the bridge
    // takes `has_AryLen = 0` and the callee compares the SECOND word of each
    // line. `Line` is `CHARACTER(MaxLineLength), INTENT(OUT)` in the callee, so
    // the buffer is the callee's to fill; the reference does not initialise its
    // own local either.
    std::vector<char> Line(static_cast<std::size_t>(MaxLineLength));
    int32_t FoundLine_l = 0;
    int CurLine = 0;
    findline_c(FileLines, n_FileLines, len_FileLines, VarName, len_VarName,
               &FoundLine_l, Line.data(), &CurLine, 0, 0);
    const bool FoundLine = (FoundLine_l != 0);

    // ! Separate line again
    // CALL GetWords ( Line, Words, 2 )
    //
    // UNCONDITIONAL, as the reference writes it -- and it is the one place this
    // unit differs structurally from `ParseDbAry_Opt`, whose `GetWords` call is
    // inside `IF (FoundLine)`. When `FoundLine` is false the reference passes a
    // `Line` that `FindLine` never assigned; `GetWords` blank-fills all
    // `NumWords` elements first, so `Words` is DEFINED either way, and every
    // read of `Words` below is inside `IF (... .AND. FoundLine)` where `Line`
    // was assigned. Reproduce the call rather than routing around it (X1).
    std::vector<char> Words(static_cast<std::size_t>(2) *
                            static_cast<std::size_t>(MaxParamLength));
    getwords_c(Line.data(), MaxLineLength, Words.data(), MaxParamLength, 2);
    const char* Words1 = Words.data();
    const char* Words2 = Words.data() + MaxParamLength;

    // ! PRINT *, "Line: ", Line     -- a COMMENT in the reference.

    // ! Print warning with default
    // IF (.NOT. FoundLine) THEN
    if (!FoundLine) {
        if (!AllowDefault_) {
            ErrVar->aviFAIL = -1;
            std::string msg(RoutineName);
            msg += ":Missing or default values are not allowed for ";
            msg.append(ftrim(VarName, len_VarName));
            msg += ". Please check control modes.";
            assign_errmsg(ErrVar, msg);
            return;
        }

        // Variable = 0     ! Default of integer inputs is 0 for now
        *Variable = 0;

        // PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input
        //          file.  Using default value of ", Variable
        print_default_warning(ftrim(VarName, len_VarName), *Variable);
    }

    // ! Debugging: show what's being read, turn into Echo later
    // IF (DEBUG_PARSING) THEN ... END IF
    // DEBUG_PARSING is `.FALSE.` as a PARAMETER; the block is dead in every
    // build of this tree and is not translated.
    if (DEBUG_PARSING) {
        // intentionally empty; see the constant above
    }

    // ! IF We haven't failed already
    // IF (ErrVar%aviFAIL >= 0 .AND. FoundLine) THEN
    if (ErrVar->aviFAIL >= 0 && FoundLine) {
        // ! Read the variable
        // READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
        //
        // The internal file is `Words(1)`, a `CHARACTER(MaxParamLength)` scalar:
        // ONE record of 200 bytes, blank-padded. One item.
        // RED TEST: the READ deleted. `Variable` keeps whatever it arrived
        // with and the status is forced to success, so both halves of the
        // reference's READ -- the transferred value and the error verdict --
        // are gone in one edit.
        (void)Words1;
        const int ErrStatLcl = 0;

        // IF ( ErrStatLcl /= 0 ) THEN
        if (ErrStatLcl != 0) {
            ErrVar->aviFAIL = -1;
            std::string msg(1, NewLine);
            msg += " >> A fatal error occurred when parsing data from \"";
            msg.append(ftrim(FileName, len_FileName));
            msg += "\".";
            msg += NewLine;
            msg += " >> The variable \"";
            msg.append(ftrim(Words2, MaxParamLength));
            msg += "\" was not assigned valid REAL value on line #";
            msg += int2lstr_trimmed(CurLine);
            msg += ".";
            msg += NewLine;
            msg += " >> The text being parsed was :";
            msg += NewLine;
            msg += "    \"";
            msg.append(ftrim(Line.data(), MaxLineLength));
            msg += "\"";
            assign_errmsg(ErrVar, msg);
        }
    }

    // IF ( PRESENT(UnEc))  THEN
    //     IF ( UnEc > 0 )  WRITE (UnEc,*)  CurLine, Tab, VarName, Tab, Variable
    // END IF
    //
    // NOT TRANSLATED, AND THE REASON IS THE TRANSLATION BOUNDARY RATHER THAN
    // THIS UNIT -- unit #54's family decision, in DECISIONS.md, and this is the
    // third of the five units carrying the identical statement. `UnEc` is a
    // Fortran UNIT NUMBER, and the record has to go to whatever file the CALLER
    // connected it to -- `<RootName>.RO.echo`, which
    // `ReadControlParameterFileSub` OPENs on it when `CntrPar%Echo > 0`
    // (ReadSetParameters.f90:358-362, clean). C++ has no access to the Fortran
    // runtime's unit table, so the only record this side could write is one to
    // `fort.<UnEc>` -- a DIFFERENT file. Writing to the wrong file is worse than
    // not writing, so nothing is emitted and the gap is named.
    //
    // THE ARM IS DEAD IN EVERY CONFIGURATION THIS CAMPAIGN TESTS, measured
    // rather than assumed: `coverage/line_coverage.json` records
    // ROSCO_Helpers.f90:268 (`IF (PRESENT(UnEc))`) 73 times per scenario and
    // :269 (`IF (UnEc > 0) WRITE`) 71 -- so the guard is EVALUATED 71 times and
    // the WRITE never runs, because all 22 `Examples/DISCON*.IN` set `Echo` to 0.
    //
    // Emitting NOTHING rather than a guarded no-op is deliberate: a translated
    // `if (UnEc > 0) { }` would be a mutable comparison no input could kill.
    // `harness/ranges.toml` holds `UnEc` at 0 for the same reason.
    (void)has_UnEc;
    (void)UnEc;
    (void)Tab;
}
