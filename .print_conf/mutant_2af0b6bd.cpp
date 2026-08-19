// VIT Translation Scaffold
// Function: ParseDbAry_Opt
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE ParseDbAry_Opt(FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 7fa7169b1346
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-18T03:53:55Z

#include <ISO_Fortran_binding.h>

#include "vit_types.h"

#include <algorithm>
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
// Copied from chkparsedata.cpp (P4), same module, same argument.
constexpr char NewLine = '\n';

// Constants.f90:37 -- `CHARACTER(1), PARAMETER :: Tab = CHAR(9)`.
constexpr char Tab = '\t';

// ROSCO_Helpers.f90:44 -- `INTEGER(IntKi), PARAMETER :: MaxLineLength = 2048`.
// `CHARACTER(MaxLineLength) :: Line` is the buffer FindLine writes into, and
// its width is also the RECORD LENGTH of the internal-file READ below: an
// internal file made from a scalar CHARACTER has exactly one record, of the
// variable's declared length, blank-padded. That is why the parser is handed a
// length rather than a NUL-terminated string.
constexpr int MaxLineLength = 2048;

// ROSCO_Helpers.f90:45 -- `INTEGER(IntKi), PARAMETER :: MaxParamLength = 200`.
// The element width of `CHARACTER(MaxParamLength), ALLOCATABLE :: Words_Ary(:)`,
// which is what `GetWords`'s bridge takes as `len_Words`.
constexpr int MaxParamLength = 200;

// `Int2LStr`'s result is `CHARACTER(11)` (ROSCO_Helpers.f90:1610). The bridge
// writes all 11 bytes, blank-padded, and does not NUL-terminate. Copied from
// chkparsedata.cpp (P4).
constexpr int Int2LStrLen = 11;

// `CHARACTER(*), PARAMETER :: RoutineName = 'ParseDbAry_Opt'`.
constexpr std::string_view RoutineName = "ParseDbAry_Opt";

// ROSCO_Helpers.f90:25 -- `LOGICAL, PARAMETER :: DEBUG_PARSING = .FALSE.`.
// A PARAMETER, so the debug block guarded by it is dead in every build of this
// tree; it is not translated and §2 of the evidence README says so. What IS
// translated is the `GetWords` call ABOVE the block, which the reference makes
// unconditionally.
constexpr bool DEBUG_PARSING = false;

// ---------------------------------------------------------------------------
// String helpers -- copied from chkparsedata.cpp (P4), same module.
// ---------------------------------------------------------------------------

// `TRIM(s)` -- drop TRAILING blanks, and blanks only. A NUL byte is not a
// blank and Fortran does not strip it; `strlen` would stop at one, which is
// why every length is carried explicitly.
std::string_view ftrim(const char* s, int len) {
    const std::string_view v(s, static_cast<std::size_t>(len));
    return v.substr(0, v.find_last_not_of(' ') + 1);
}

// `ErrVar%ErrMsg = <expr>` REALLOCATES to exactly LEN of the right-hand side.
// Copied byte for byte from chkparsedata.cpp, which copied it from
// checkinputs.cpp (P4) -- including the `memset` to NUL past the new length,
// which unit #29 measured in both directions.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: ParseDbAry_Opt: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: ParseDbAry_Opt: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// `READ (Line,*,IOSTAT=ErrStatLcl) Ary`
//
// LIST-DIRECTED INPUT OF REAL(8) FROM AN INTERNAL FILE. The reference reads
// SIZE(Ary) values out of one blank-padded record and branches on whether the
// read succeeded; on failure it reports and returns, leaving `Ary` in whatever
// state the partial transfer left it. Both halves are outputs of this unit --
// the error message AND the array -- so the model has to reproduce which items
// were assigned, not just the verdict.
//
// EVERY RULE BELOW IS MEASURED, not read off the standard.
// `evidence/ParseDbAry_Opt/fortran_io_probe.f90` runs 31 records through the
// reference's own `READ` with a `-987.654` sentinel in every element, and
// `evidence/ParseDbAry_Opt/fortran_io_probe.txt` is what gfortran printed.
// The rows that fixed a decision:
//
//   '1.5   NameHere'      n=2  iostat=5010  1.5, SENTINEL
//        -> a value already transferred STAYS transferred after a later item
//           fails. The failing item and everything after it is untouched.
//   '1.0 2.0'             n=4  iostat=  -1  1.0, 2.0, SENTINEL, SENTINEL
//        -> running out of record is an END condition (negative), not an error,
//           and it is still `/= 0`, which is all the reference tests.
//   '1.0, ,3.0 Name'      n=3  iostat=   0  1.0, SENTINEL, 3.0
//        -> a null value leaves the item UNCHANGED and consumes an item.
//   '2* 8.5 Name'         n=3  iostat=   0  SENTINEL, SENTINEL, 8.5
//        -> `r*` followed by a separator is r NULL values.
//   '1.0 / 2.0 Name'      n=3  iostat=   0  1.0, SENTINEL, SENTINEL
//        -> `/` terminates the transfer and is NOT an error.
//   '1.0;2.0'             n=2  iostat=5010  1.0, SENTINEL
//        -> ';' TERMINATES a value (so 1.0 is stored) and then fails, which is
//           why it is in the separator set below and rejected right after.
//   '1.0.0 2.0'           n=1  iostat=5010  SENTINEL
//        -> a character that is neither part of the number nor a separator
//           fails the item, and nothing is stored for it. Contrast with ';'.
//   '1.5d2', '1.5q2', '1.5+2', '.5', '5.', 'Infinity', '-inf', 'NaN'
//        -> all accepted; 'bad', '!', "'1.0'", '*1.0', '0*1.0', '+ 1.0',
//           '1.0e' all give 5010.
//
// The three iostat values are reproduced as gfortran spells them (0, 5010, -1)
// even though the reference only ever tests `/= 0`: a model that returned "1"
// for both failure kinds would be a model of the TEST rather than of the READ,
// and the next unit in this family reads the value.
// ---------------------------------------------------------------------------

// libgfortran's `is_separator`, transcribed: the characters that legally END a
// list-directed value. ';' is in the set -- see the '1.0;2.0' row above, which
// is the case that distinguishes "in the set" from "not in the set".
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
//
// libgfortran does not decide "is this a number" and then convert. It scans a
// field, and if it reaches a state where the field COULD have ended -- which a
// decimal point is enough to reach, because `1.` is a legal real -- it calls
// its conversion on what it has. `strtod` of a field with no digits yields 0.0,
// and THAT VALUE IS TRANSFERRED before the error is raised. Without the point
// the scanner rejects the field before converting anything and the item keeps
// the value it had.
//
// MEASURED, twenty records, `evidence/ParseDbAry_Opt/fortran_io_probe2.f90`:
//
//     '+ 1.0'  '- 1.0'  '+/1.0'  '+,1.0'  '-'  '+'  'e1 1.0'  '.. 1.0'
//     '-.e 1.0'                                    iostat 5010, NOT stored
//     '. 1.0'  '-. 1.0'  '+. 1.0'  './\'  '-./\'  '.,1.0'  '.'
//     '.e1 1.0'  '.d0 1.0'                         iostat 5010, STORED 0.0
//
// So the discriminator is the decimal point, and it is the point ALONE: an
// exponent may follow it (`.e1`, `.d0` both store), but a MALFORMED exponent
// (`-.e ` -- the letter with no digits after it) rejects before converting, and
// so does a second point (`..`, and `1.0.0` in the first probe).
//
// THIS COST THREE CASES OF 13,674 AND THEY WERE FOUND BY THE HARNESS, not by
// reading libgfortran. The first version of this parser returned one failure
// kind and stored nothing; `evidence/ParseDbAry_Opt/harness.parser-no-zero-store.json`
// is that run. The three cases are the ones where `Ary` arrived ALLOCATED, so
// every element had an oracle and the disagreement was visible; see §3 of the
// evidence README.
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
// behaviour and is what makes `Ary` an output on the failure path too.
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
// gfortran's LIST-DIRECTED OUTPUT, for the two records this unit writes.
//
// `list_directed_real` is copied from `powercontrolsetpoints.cpp` (P4), where
// it is measured against 22,526 records written by the reference's own runtime
// (`evidence/PowerControlSetpoints/ld_probe.txt`). Nothing about it is
// re-derived here.
//
// The RECORD LAYOUT is measured for this unit's own two statements, in
// `evidence/ParseDbAry_Opt/fortran_io_probe.{f90,txt}`:
//
//   PRINT *, "...["    , Ary(3) all zero , "]"
//     | ROSCO Warning: ... value of [   0.0000000000000000        0.0000...
//       ...0000000        0.0000000000000000      ]
//     -> one leading blank starts the record; a CHARACTER item is written raw;
//        each REAL is a self-contained 26-byte field (21 right-justified + 5
//        trailing blanks) with NO separator between consecutive reals; and ONE
//        separator blank appears before a CHARACTER item that follows a real --
//        which is the six blanks before the `]` against five after the reals in
//        the WRITE record.
//
//   WRITE (77,*) LineNum, Tab, ParamName, Tab, Ary
//     |          72 \tIPC_Vramp<pad to 200>\t   9.1199999999999992
//       ...        11.400000000000000     |
//     -> the leading INTEGER(4) is right-justified in 12; one separator blank
//        before the CHARACTER that follows it; NO separator between two
//        CHARACTER items, nor between a CHARACTER and a following real.
//
// NEITHER RECORD IS COMPARED BY ANY LAYER of this unit's evidence -- the
// differential harness compares out-parameters, the gate compares simulation
// channels, and both of these are side effects on unit 6 and on unit `UnEc`.
// They are here because the contract is `mirror` and the PRINT is LIVE: 66
// executions across 21 of the 27 scenarios (`coverage/line_coverage.json`,
// ROSCO_Helpers.f90:946). Dropping it would delete a warning the shipped
// controller prints. §7 of the evidence README says exactly what is and is not
// checked about them.
// ---------------------------------------------------------------------------

// Right-justify `text` in a field of `w`, or fill with '*' on overflow.
// Copied from powercontrolsetpoints.cpp / debug.cpp (P4).
std::string field(const std::string& text, int w) {
    if (static_cast<int>(text.size()) > w) {
        return std::string(static_cast<std::size_t>(w), '*');
    }
    return std::string(static_cast<std::size_t>(w) - text.size(), ' ') + text;
}

// gfortran renders a non-finite value as a WORD. Copied from
// powercontrolsetpoints.cpp (P4).
bool nonfinite_text(double v, std::string& out) {
    if (!(std::isnan(v))) { out = "NaN"; return true; }
    if (std::isinf(v)) {
        out = std::signbit(v) ? "-Infinity" : "Infinity";
        return true;
    }
    return false;
}

// One list-directed field for a REAL(8). Copied from
// powercontrolsetpoints.cpp (P4).
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

// `PRINT *, "ROSCO Warning: ..."//TRIM(ParamName)//"...[", Ary, "]"` on unit 6.
void print_default_warning(std::string_view ParamName, const double* Ary, int n) {
    std::string rec(1, ' ');
    rec += "ROSCO Warning: Did not find correct size ";
    rec.append(ParamName);
    rec += " in input file.  Using default value of [";
    for (int i = 0; i < n; ++i) {
        rec += list_directed_real(Ary[i]);
    }
    rec += ' ';
    rec += ']';
    rec += '\n';
    std::fwrite(rec.data(), 1, rec.size(), stdout);
}

// ---------------------------------------------------------------------------
// `Ary` -- the caller's own descriptor for a `REAL(DbKi), ALLOCATABLE (:)`.
// ---------------------------------------------------------------------------

bool ary_allocated(const CFI_cdesc_t* d) { return d->base_addr != nullptr; }

CFI_index_t ary_size(const CFI_cdesc_t* d) {
    return ary_allocated(d) ? d->dim[0].extent : 0;
}

// Fortran subscripts, so element `i` runs from `lower_bound`.
double* ary_at(CFI_cdesc_t* d, CFI_index_t i) {
    CFI_index_t s = d->dim[0].lower_bound + i;
    return static_cast<double*>(CFI_address(d, &s));
}

}  // namespace

void ParseDbAry_Opt(char* FileLines, int n_FileLines, int len_FileLines,
                    char* ParamName, int len_ParamName,
                    CFI_cdesc_t* Ary, int AryLen,
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

    // CALL FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)
    //
    // `Line` is `CHARACTER(MaxLineLength), INTENT(OUT)` in the callee, so the
    // buffer is the callee's to fill; it is NOT blank-initialised here, because
    // the reference does not initialise its own local either and every path
    // that reads `Line` is inside `IF (FoundLine)`, where FindLine has assigned
    // it. Reproduce that, do not improve on it (P7).
    std::vector<char> Line(static_cast<std::size_t>(MaxLineLength));
    int32_t FoundLine_l = 0;
    int LineNum = 0;
    findline_c(FileLines, n_FileLines, len_FileLines, ParamName, len_ParamName,
               &FoundLine_l, Line.data(), &LineNum, 1, AryLen);
    const bool FoundLine = (FoundLine_l != 0);

    // ! Minimum array length
    // IF (AryLen < 1) THEN ; FinalAryLen = 1 ; ELSE ; FinalAryLen = AryLen ; ENDIF
    int FinalAryLen;
    if (AryLen < 1) {
        FinalAryLen = 1;
    } else {
        FinalAryLen = AryLen;
    }

    // ! Allocate array and handle errors
    // ALLOCATE ( Ary(FinalAryLen) , STAT=ErrStatLcl )
    //
    // THE REFERENCE DOES NOT DEALLOCATE FIRST, and that is the whole of this
    // arm: `ALLOCATE` on an object that is already allocated is an ERROR in
    // Fortran, so a caller who hands in an allocated `Ary` gets a non-zero
    // STAT, keeps the array it arrived with -- data, extent and bounds -- and
    // falls into the branch whose message says so. The `ALLOCATED(Ary)` test
    // that picks between the two messages is therefore TRUE on exactly the
    // reachable failure and FALSE only on a genuine allocation failure.
    int ErrStatLcl = 0;
    const bool was_allocated = ary_allocated(Ary);
    if (was_allocated) {
        ErrStatLcl = 5014;   // gfortran: "Attempt to allocate an allocated object"
    } else {
        CFI_index_t lo = 1;
        CFI_index_t hi = FinalAryLen;
        ErrStatLcl = CFI_allocate(Ary, &lo, &hi, 0);
    }
    if (ErrStatLcl != 0) {
        if (ary_allocated(Ary)) {
            ErrVar->aviFAIL = -1;
            std::string msg(RoutineName);
            msg += ":Error allocating memory for the ";
            msg.append(ftrim(ParamName, len_ParamName));
            msg += " array; array was already allocated.";
            assign_errmsg(ErrVar, msg);
        } else {
            ErrVar->aviFAIL = -1;
            std::string msg(RoutineName);
            msg += ":Error allocating memory for ";
            msg += int2lstr_trimmed(AryLen);
            msg += " characters in the ";
            msg.append(ftrim(ParamName, len_ParamName));
            msg += " array.";
            assign_errmsg(ErrVar, msg);
        }
    }

    const int n = static_cast<int>(ary_size(Ary));

    // ! Print warning with default
    // IF (.NOT. FoundLine) THEN
    if (!FoundLine) {
        if (!AllowDefault_) {
            ErrVar->aviFAIL = -1;
            std::string msg(RoutineName);
            msg += ":Missing or default values are not allowed for ";
            msg.append(ftrim(ParamName, len_ParamName));
            msg += ". Please check control modes and array length.";
            assign_errmsg(ErrVar, msg);
            // RETURN -- and the reference does NOT call Cleanup here. It cannot
            // leak: `Words_Ary` is allocated further down, inside the
            // `IF (FoundLine)` block this arm is the negation of.
            return;
        }

        // Ary = 0     ! Default of allocatable arrays is 0 for now
        for (int i = 0; i < n; ++i) {
            *ary_at(Ary, i) = 0.0;
        }

        // PRINT *, "ROSCO Warning: ..."
        std::vector<double> values(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            values[static_cast<std::size_t>(i)] = *ary_at(Ary, i);
        }
        print_default_warning(ftrim(ParamName, len_ParamName), values.data(), n);
    }

    // IF (FoundLine) THEN
    if (FoundLine) {
        // ! Allocate words array
        // ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
        //
        // `Words_Ary` is `CHARACTER(MaxParamLength), ALLOCATABLE (:)` and is
        // read by NOTHING outside the DEBUG_PARSING block, which is dead. It is
        // still allocated and still passed to `GetWords`, because that is what
        // the reference does and X1 forbids routing around a callee. The vector
        // below IS the allocation -- `MaxParamLength` bytes per element, the
        // same layout `GetWords`'s bridge expects -- so the call is the
        // reference's call and not a re-implementation of it.
        //
        // THE `STAT=` BRANCH IS NOT TRANSLATED. `std::vector` reports failure
        // by throwing, not by a status, so there is no `ErrStatLcl` to test --
        // and the branch it guards needs a genuine out-of-memory, which no
        // input in this unit's domain produces (`AryLen` is pinned to
        // [0, 32], so the request is at most 33 x 200 bytes). Writing
        // `ErrStatLcl = 0; if (ErrStatLcl != 0)` would put a comparison in the
        // translation that no input can make either way -- the shape units #1,
        // #4 and #43 each measured as surviving mutants. Named in §7.
        const int NumWords = AryLen + 1;
        std::vector<char> Words_Ary(
            static_cast<std::size_t>(std::max(NumWords, 0))
            * static_cast<std::size_t>(MaxParamLength));

        // ! Separate line string into AryLen + 1 words, should include variable name
        // CALL GetWords ( Line, Words_Ary, AryLen + 1 )
        getwords_c(Line.data(), MaxLineLength, Words_Ary.data(), MaxParamLength, NumWords);

        // ! Debug Output -- IF (DEBUG_PARSING) THEN ... END IF
        // DEBUG_PARSING is `.FALSE.` as a PARAMETER; the block is dead in every
        // build of this tree and is not translated.
        if (DEBUG_PARSING) {
            // intentionally empty; see the constant above
        }

        // ! Read array
        // READ (Line,*,IOSTAT=ErrStatLcl)  Ary
        std::vector<double> values(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            values[static_cast<std::size_t>(i)] = *ary_at(Ary, i);
        }
        ErrStatLcl = list_read_reals(Line.data(), MaxLineLength, values.data(), n);
        for (int i = 0; i < n; ++i) {
            *ary_at(Ary, i) = values[static_cast<std::size_t>(i)];
        }

        if (ErrStatLcl != 0) {
            ErrVar->aviFAIL = -1;
            std::string msg(RoutineName);
            msg += ":A fatal error occurred when parsing data from \"";
            msg.append(ftrim(FileName, len_FileName));
            msg += "\".";
            msg += NewLine;
            msg += " >> The \"";
            msg.append(ftrim(ParamName, len_ParamName));
            msg += "\" array was not assigned valid REAL values on line #";
            msg += int2lstr_trimmed(LineNum);
            msg += ".";
            msg += NewLine;
            msg += " >> The text being parsed was :";
            msg += NewLine;
            msg += "    \"";
            msg.append(ftrim(Line.data(), MaxLineLength));
            msg += "\"";
            assign_errmsg(ErrVar, msg);
            // RETURN, and the `CALL Cleanup()` the reference writes on the NEXT
            // line is unreachable Fortran -- a statement after an unconditional
            // RETURN. Transcribed as unreachable: the vector's destructor is
            // what actually frees `Words_Ary`, so the reference's leak is not
            // reproduced. Recorded in §7 of the evidence README rather than
            // silently repaired.
            return;
        }
    }

    // IF ( PRESENT(UnEc))  THEN
    //     IF ( UnEc > 0 )  WRITE (UnEc,*)  LineNum, Tab, ParamName, Tab, Ary
    // END IF
    //
    // NOT TRANSLATED, AND THE REASON IS THE TRANSLATION BOUNDARY RATHER THAN
    // THIS UNIT. `UnEc` is a Fortran UNIT NUMBER, and the record has to go to
    // whatever file the CALLER connected it to -- `<RootName>.RO.echo`, which
    // `ReadControlParameterFileSub` OPENs on it when `CntrPar%Echo > 0`
    // (ReadSetParameters.f90:358-362, clean). C++ has no access to the Fortran
    // runtime's unit table, so the only record this side could write is one to
    // `fort.<UnEc>` -- a DIFFERENT file from the one the reference writes,
    // whenever the arm is live at all. Writing to the wrong file is worse than
    // not writing, so nothing is emitted and the gap is named.
    //
    // THE ARM IS DEAD IN EVERY CONFIGURATION THIS CAMPAIGN TESTS, measured
    // rather than assumed: `CntrPar%Echo > 0` is FALSE in all 27 scenarios
    // (`coverage/line_coverage.json`, ReadSetParameters.f90:359 reached 28
    // times, :360 zero), so `UnEc` is 0 at every one of this unit's 1,232
    // calls and the guard is false at all of them. All 22 `Examples/DISCON*.IN`
    // set `Echo` to 0.
    //
    // Emitting NOTHING rather than a guarded no-op is deliberate: a translated
    // `if (UnEc > 0) { }` would be a mutable comparison no input could kill.
    // `harness/ranges.toml` holds `UnEc` at 0 for the same reason, so the
    // reference does not write a record the translation has no counterpart for.
    // Raised in DECISIONS.md -- the four sibling `Parse*_Opt` units carry the
    // identical statement, so this is a family decision, not a local one.
    (void)has_UnEc;
    (void)UnEc;
    (void)LineNum;
    (void)Tab;

    // CALL Cleanup() -- deallocates Words_Ary, which is out of scope by here.
}
