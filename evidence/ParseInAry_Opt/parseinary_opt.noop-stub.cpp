// VIT Translation Scaffold
// Function: ParseInAry_Opt
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE ParseInAry_Opt(FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: ee7c19a2c357
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-18T06:20:19Z

#include <ISO_Fortran_binding.h>

#include "vit_types.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// THE SIBLING UNIT. `ParseDbAry_Opt` (#54) and `ParseInAry_Opt` (#55) are the
// SAME SUBROUTINE with one declaration changed. `diff` of the two clean bodies
// at ROSCO_Helpers.f90:741-876 and :882-1015 is: the routine name, the
// declaration of `Ary` (`INTEGER(IntKi)` here, `REAL(DbKi)` there), the
// declaration ORDER of three locals, one commented-out PRINT, and whitespace.
// Nothing else.
//
// So everything that is not the item type is COPIED from
// `translations/ROSCO_Helpers/parsedbary_opt.cpp` (P4) rather than re-derived,
// and everything that IS the item type is MEASURED here rather than adapted
// from that file's REAL measurements. The two readers are not the same reader:
// the REAL one stores a converted 0.0 for a field with a decimal point and no
// digits, and the INTEGER one stores nothing on any failure. Section "the
// READ" below has the records.
// ---------------------------------------------------------------------------

// SysGnuLinux.f90:37 -- `CHARACTER(*), PARAMETER :: NewLine = ACHAR(10)`.
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
// writes all 11 bytes, blank-padded, and does not NUL-terminate.
constexpr int Int2LStrLen = 11;

// `CHARACTER(*), PARAMETER :: RoutineName = 'ParseInAry_Opt'`.
constexpr std::string_view RoutineName = "ParseInAry_Opt";

// ROSCO_Helpers.f90:25 -- `LOGICAL, PARAMETER :: DEBUG_PARSING = .FALSE.`.
// A PARAMETER, so the debug block guarded by it is dead in every build of this
// tree; it is not translated. What IS translated is the `GetWords` call ABOVE
// the block, which the reference makes unconditionally.
constexpr bool DEBUG_PARSING = false;

// ---------------------------------------------------------------------------
// String helpers -- copied from parsedbary_opt.cpp (P4), which copied them from
// chkparsedata.cpp, which copied `assign_errmsg` from checkinputs.cpp.
// ---------------------------------------------------------------------------

// `TRIM(s)` -- drop TRAILING blanks, and blanks only. A NUL byte is not a
// blank and Fortran does not strip it; `strlen` would stop at one, which is
// why every length is carried explicitly.
std::string_view ftrim(const char* s, int len) {
    const std::string_view v(s, static_cast<std::size_t>(len));
    return v.substr(0, v.find_last_not_of(' ') + 1);
}

// `ErrVar%ErrMsg = <expr>` REALLOCATES to exactly LEN of the right-hand side.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: ParseInAry_Opt: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: ParseInAry_Opt: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// LIST-DIRECTED INPUT OF INTEGER(4) FROM AN INTERNAL FILE. The reference reads
// SIZE(Ary) values out of one blank-padded record and branches on whether the
// read succeeded; on failure it reports and returns, leaving `Ary` in whatever
// state the partial transfer left it. Both halves are outputs of this unit --
// the error status decides the message, the array decides everything else -- so
// the model has to reproduce WHICH ITEMS WERE ASSIGNED, not just the verdict.
//
// EVERY RULE BELOW IS MEASURED, none of it read off the standard and none of it
// carried over from the sibling. 113 records through the reference's own READ
// with a `-987654` sentinel in every element:
//
//     evidence/ParseInAry_Opt/fortran_io_probe.f90   / .txt   (human-readable)
//     evidence/ParseInAry_Opt/fortran_io_probe2.f90  / .txt
//     evidence/ParseInAry_Opt/parser_conformance.f90 / .txt   (machine-readable)
//     evidence/ParseInAry_Opt/parser_conformance.cpp          (the replay)
//
// The third pair is the one that matters: it emits each record as ICHAR codes
// beside gfortran's iostat and every element, and `parser_conformance.cpp`
// replays all 113 through `list_read_ints` below and compares. The parser is
// therefore CHECKED against the reference rather than argued to match it.
//
// THE ONE RULE THE SIBLING HAS AND THIS ONE DOES NOT. Unit #54 found that a
// REAL field with a decimal point and no digits reaches `strtod`, whose 0.0 is
// TRANSFERRED before the error is raised -- three cases of 13,674 turned on it.
// The INTEGER reader has no such path, and it was looked for rather than
// assumed absent:
//
//     '. 5'  '., 5'  './ 5'  '+. 5'  '5. 6'  '5., 6'  'e1 5'  '.e1 5'
//         -> iostat 5010, EVERY element still at the sentinel
//
// Every one of the 113 records that returns 5010 leaves the failing item
// unchanged. Items COMPLETED before it keep their values ('1 2 bad 4' -> 1, 2,
// sentinel, sentinel), which is the other half of the same statement.
//
// AND THE ONE RULE THAT IS THIS UNIT'S OWN: the semicolon. Unit #54 measured
// '1.0;2.0' as storing 1.0 and then failing, and put ';' in its separator set.
// The INTEGER reader is not that:
//
//     '1;2'    -> 5010, NOTHING stored     ';' does not terminate a value
//     '1 ;2'   -> 0,    [1, unchanged]     ';' after a BLANK is a null value
//     '1, ;2'  -> 5010, [1, unchanged]     ... but not after a COMMA
//     ';1'     -> 5010                     ... and not at the first item
//     '1<LF>;2'-> 5010, [1, unchanged]     ... and not after an end-of-line
//     '1 ;2 3' -> 0,    [1, unchanged, 2]  it CONSUMES an item and continues
//     '1 ;;2'  -> 5010, [1, unchanged]     ... and then behaves like a comma
//
// which is the `comma_flag` state below. Thirty-one semicolon records, because
// one wrong branch here is a wrong `Ary` and a wrong `ErrMsg` on the same case.
// ---------------------------------------------------------------------------

// Which separator was consumed to get to the current item. `;` is legal at the
// start of an item only when the answer is `Blank` -- measured above.
enum class Sep { Blank, Comma, Eol };

// A BLANK, for the purpose of skipping. Fortran's list input treats the tab as
// a blank; '2*3.5' vs '2* 8' turns on it and both are in the corpus.
bool is_blank(char c) { return c == ' ' || c == '\t'; }

bool is_eol(char c) { return c == '\n' || c == '\r'; }

bool is_digit(char c) { return c >= '0' && c <= '9'; }

// The characters that legally END an integer value. ';' IS NOT ONE -- '1;2'
// returns 5010 having stored nothing, which is the record that distinguishes
// this set from the sibling's.
bool is_value_terminator(char c) {
    return is_blank(c) || is_eol(c) || c == ',' || c == '/';
}

// libgfortran's repeat-count ceiling, MEASURED at the boundary rather than
// recalled from its source: '199999999*7' and '200000000*7' both give 7 7,
// '200000001*7' gives 5010.
constexpr long MaxRepeat = 200000000;

// Parse ONE integer starting at `p`. Returns false on any failure, in which
// case NOTHING is stored -- which is the whole difference from the sibling's
// three-valued `RealParse`. The TERMINATOR is not checked here; that is the
// caller's job, because '5abc' and '5 abc' differ only in it.
bool parse_int(const char* rec, int len, int& p, int32_t& out) {
    bool neg = false;
    if (p < len && (rec[p] == '+' || rec[p] == '-')) {
        neg = (rec[p] == '-');
        ++p;
    }
    int digits = 0;
    long long acc = 0;
    bool overflow = false;
    while (p < len && is_digit(rec[p])) {
        ++digits;
        if (!overflow) {
            acc = acc * 10 + (rec[p] - '0');
            // The bound is on the VALUE, not on the digit count:
            // '0000000000000000042' is 42 and reads clean, while
            // '002147483648' is 5010. Stop accumulating once the magnitude can
            // no longer matter, so a 20-digit record cannot overflow `acc`.
            if (acc > 4294967296LL) {
                overflow = true;
            }
        }
        ++p;
    }
    if (digits == 0) {
        return false;
    }
    const long long v = neg ? -acc : acc;
    if (overflow || v > 2147483647LL || v < -2147483648LL) {
        return false;
    }
    out = static_cast<int32_t>(v);
    return true;
}

// The transfer itself. `v` holds the items on entry, and every item this
// returns without assigning keeps the value it arrived with -- which is the
// reference's behaviour and is what makes `Ary` an output on the failure path
// too. The return value is gfortran's own iostat: 0, 5010 or -1 (end of
// record). The reference only tests `/= 0`; the three are reproduced as
// gfortran spells them because a model of the TEST is not a model of the READ,
// and the three sibling units in this family read the value too.
int list_read_ints(const char* rec, int len, int32_t* v, int n) {
    int p = 0;
    int i = 0;
    Sep sep = Sep::Blank;
    bool first_item = true;

    // Blanks before the first item. '     42 Name' reads 42.
    while (p < len && (is_blank(rec[p]) || is_eol(rec[p]))) {
        ++p;
    }

    // Consume whatever separates two items: blanks and end-of-lines, then at
    // most one comma. Records which kind was crossed, because the semicolon
    // rule reads it.
    const auto eat_separator = [&]() {
        bool crossed_eol = false;
        while (p < len && (is_blank(rec[p]) || is_eol(rec[p]))) {
            if (is_eol(rec[p])) {
                crossed_eol = true;
            }
            ++p;
        }
        if (p < len && rec[p] == ',') {
            ++p;
            while (p < len && (is_blank(rec[p]) || is_eol(rec[p]))) {
                ++p;
            }
            sep = Sep::Comma;
        } else {
            sep = crossed_eol ? Sep::Eol : Sep::Blank;
        }
    };

    while (i < n) {
        if (p >= len) {
            return -1;                          // END: the record ran out
        }

        if (rec[p] == '/') {                    // terminate; the rest unchanged
            return 0;
        }

        if (rec[p] == ',') {                    // a null value: item unchanged
            ++p;
            ++i;
            first_item = false;
            while (p < len && (is_blank(rec[p]) || is_eol(rec[p]))) {
                ++p;
            }
            sep = Sep::Comma;
            continue;
        }

        if (rec[p] == ';') {
            // Legal only at the start of an item that was reached across
            // BLANKS, and never as the first item. Measured, thirty-one
            // records; see the table above.
            if (first_item || sep != Sep::Blank) {
                return 5010;
            }
            ++p;
            ++i;
            while (p < len && (is_blank(rec[p]) || is_eol(rec[p]))) {
                ++p;
            }
            sep = Sep::Comma;                   // '1 ;;2' is 5010
            continue;
        }

        first_item = false;

        // An optional repeat count `r*`, r a positive integer no greater than
        // MaxRepeat. '02*7' is 7 7, so leading zeros are part of the count.
        long repeat = 1;
        int q = p;
        while (q < len && is_digit(rec[q])) {
            ++q;
        }
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
            // `r*` with a separator or the end of the record after it is r
            // NULL values: '2* 8 Name' leaves two items and reads 8 into the
            // third, and '3*' leaves three and succeeds.
            if (p >= len || is_value_terminator(rec[p]) || rec[p] == ';') {
                for (long k = 0; k < repeat && i < n; ++k) {
                    ++i;
                }
                eat_separator();
                continue;
            }
        }

        int32_t value = 0;
        if (!parse_int(rec, len, p, value)) {
            return 5010;
        }
        if (p < len && !is_value_terminator(rec[p])) {
            return 5010;                        // '5abc', '1.5', '1e2', '1;2'
        }
        for (long k = 0; k < repeat && i < n; ++k) {
            v[i++] = value;
        }
        eat_separator();
    }
    return 0;
}

// ---------------------------------------------------------------------------
// gfortran's LIST-DIRECTED OUTPUT of INTEGER(4), for the one record this unit
// writes to a unit C++ can reach.
//
// MEASURED for this unit's own statement, in
// `evidence/ParseInAry_Opt/fortran_io_probe.{f90,txt}`:
//
//   PRINT *, "...["   , Ary(3) all zero , "]"
//     | ROSCO Warning: ... value of [           0           0           0 ]
//     -> one leading blank starts the record; a CHARACTER item is written raw;
//        each INTEGER(4) is a self-contained 12-byte field, right-justified,
//        with NO separator between consecutive integers or between a CHARACTER
//        and a following integer; and ONE separator blank appears before a
//        CHARACTER item that FOLLOWS an integer -- which is the blank between
//        the last `0` and the `]`.
//
//   PRINT *, "[", (/0, -1, 2147483647, -2147483647/), "]"
//     | [           0          -1  2147483647 -2147483647 ]
//     -> the width is 12 for every value in the type's range; the widest
//        INTEGER(4) is `-2147483648` at 11 characters, so the field never
//        overflows and there is no '*' fill to model.
//
// The record is LIVE in ROSCO -- but NOT in this unit and NOT in this
// campaign's 27 scenarios. `coverage/line_coverage.json` records
// ROSCO_Helpers.f90:806 and :807 (the `Ary = 0` default and this PRINT) with NO
// counter at all against 230 executions of :775, so `FoundLine` is TRUE at
// every one of this unit's calls in every scenario. That is a real difference
// from the sibling, whose same two lines run 66 times across 21 scenarios. It
// is translated because the contract is `mirror` and a user with a shorter
// input file reaches it; §8 of the evidence README says which layers can see it
// (the differential harness, and only there).
// ---------------------------------------------------------------------------

// Right-justify `text` in a field of `w`. Copied from parsedbary_opt.cpp (P4).
// The '*' overflow fill that file carries is unreachable for INTEGER(4) in a
// 12-wide field and is not reproduced; the fill exists in gfortran, but no
// value of this type can select it.
std::string field(const std::string& text, int w) {
    if (static_cast<int>(text.size()) > w) {
        return std::string(static_cast<std::size_t>(w), '*');
    }
    return std::string(static_cast<std::size_t>(w) - text.size(), ' ') + text;
}

// One list-directed field for an INTEGER(4).
std::string list_directed_int(int32_t v) {
    char tmp[32];
    std::snprintf(tmp, sizeof tmp, "%d", static_cast<int>(v));
    return field(std::string(tmp), 12);
}

// `PRINT *, "ROSCO Warning: ..."//TRIM(ParamName)//"...[", Ary, "]"` on unit 6.
void print_default_warning(std::string_view ParamName, const int32_t* Ary, int n) {
    std::string rec(1, ' ');
    rec += "ROSCO Warning: Did not find correct size ";
    rec.append(ParamName);
    rec += " in input file.  Using default value of [";
    for (int i = 0; i < n; ++i) {
        rec += list_directed_int(Ary[i]);
    }
    rec += ' ';
    rec += ']';
    rec += '\n';
    std::fwrite(rec.data(), 1, rec.size(), stdout);
}

// ---------------------------------------------------------------------------
// `Ary` -- the caller's own descriptor for an `INTEGER(IntKi), ALLOCATABLE (:)`.
// ---------------------------------------------------------------------------

bool ary_allocated(const CFI_cdesc_t* d) { return d->base_addr != nullptr; }

CFI_index_t ary_size(const CFI_cdesc_t* d) {
    return ary_allocated(d) ? d->dim[0].extent : 0;
}

// Fortran subscripts, so element `i` runs from `lower_bound`.
int32_t* ary_at(CFI_cdesc_t* d, CFI_index_t i) {
    CFI_index_t s = d->dim[0].lower_bound + i;
    return static_cast<int32_t*>(CFI_address(d, &s));
}

}  // namespace

void ParseInAry_Opt(char* FileLines, int n_FileLines, int len_FileLines,
                    char* ParamName, int len_ParamName,
                    CFI_cdesc_t* Ary, int AryLen,
                    char* FileName, int len_FileName,
                    errorvariables_view_t* ErrVar,
                    int has_AllowDefault, int32_t AllowDefault,
                    int has_UnEc, int UnEc) {
    // RED TEST -- the whole unit removed. Predicted to fail 13,448 of 13,674:
    // the 226 that pass are exactly the cases that arrived with
    // ErrVar%aviFAIL < 0, where the reference's own first statement returns.
    (void)FileLines; (void)n_FileLines; (void)len_FileLines;
    (void)ParamName; (void)len_ParamName; (void)Ary; (void)AryLen;
    (void)FileName; (void)len_FileName; (void)ErrVar;
    (void)has_AllowDefault; (void)AllowDefault; (void)has_UnEc; (void)UnEc;
}
