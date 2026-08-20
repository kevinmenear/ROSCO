// VIT Translation Scaffold
// Function: ParseInput_Int_Opt
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE ParseInput_Int_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: dcfa4936bfaf
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-20T03:04:13Z

#include "vit_types.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// THIS UNIT IS `ParseInput_Dbl_Opt` (#56) WITH ONE DECLARATION CHANGED, AND THE
// TWO HALVES OF IT COME FROM TWO DIFFERENT SIBLINGS.
//
// `diff` of the two clean bodies at ROSCO_Helpers.f90:115-192 and :196-274
// (baseline 54dd134) is: the routine name, the declaration of `Variable`
// (`INTEGER(IntKi)` here, `REAL(DbKi)` there), the word INTEGER/REAL in the
// parse-error message, and whitespace. Nothing else -- not the control flow,
// not the callee calls, not the arms.
//
// So the CALLER half -- constants, `ftrim`, `assign_errmsg`, `int2lstr_trimmed`
// and the whole subroutine body -- is copied from
// `translations/ROSCO_Helpers/parseinput_dbl_opt.cpp` (P4).
//
// And the ITEM half -- the list-directed READ and the list-directed OUTPUT
// field -- is copied from `translations/ROSCO_Helpers/parseinary_opt.cpp` (P4),
// which is the unit that MEASURED both against gfortran for an INTEGER(4) item
// (113 records through the reference's own READ,
// `evidence/ParseInAry_Opt/parser_conformance.{f90,cpp,txt}`).
//
// THE SPLIT IS THE RUNBOOK'S OWN DISCRIMINATOR: item TYPE against record
// GRAMMAR. Unit #55 broke by copying unit #54's separator set, which is about
// the item type (';' terminates a REAL and not an INTEGER); unit #56 held by
// deriving its output record from measured grammar rules. Here the item type
// changed, so every item-typed rule comes from the INTEGER sibling and none
// from the REAL one, and the grammar rule that is genuinely this unit's own --
// a CHARACTER item followed by ONE INTEGER and nothing after -- is PRICED
// against gfortran before the corpus is generated
// (`evidence/ParseInput_Int_Opt/record_form_probe.{f90,cpp,txt}`).
//
// The block hashes are in `evidence/ParseInput_Int_Opt/p4_blocks.txt`, written
// by `check_p4_blocks.py`, which re-extracts each block from its source file
// and compares bytes.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Constants the reference names, read off the source rather than recalled.
// ---------------------------------------------------------------------------

// SysGnuLinux.f90:37 -- `CHARACTER(*), PARAMETER :: NewLine = ACHAR(10)`.
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
// 2048-byte `Line`, and it is shared with `ParseInput_Dbl_Opt`.
constexpr int MaxParamLength = 200;

// `Int2LStr`'s result is `CHARACTER(11)` (ROSCO_Helpers.f90:1610). The bridge
// writes all 11 bytes, blank-padded, and does not NUL-terminate.
constexpr int Int2LStrLen = 11;

// `CHARACTER(*), PARAMETER :: RoutineName = 'ParseInput_Int_Opt'`.
constexpr std::string_view RoutineName = "ParseInput_Int_Opt";

// ROSCO_Helpers.f90:25 -- `LOGICAL, PARAMETER :: DEBUG_PARSING = .FALSE.`.
// A PARAMETER, so the debug block guarded by it is dead in every build of this
// tree; it is not translated and the evidence README says so. What IS
// translated is the `GetWords` call ABOVE the block, which the reference makes
// unconditionally.
constexpr bool DEBUG_PARSING = false;

// ---------------------------------------------------------------------------
// String helpers -- copied from parseinput_dbl_opt.cpp (P4), same module.
// ---------------------------------------------------------------------------

// `TRIM(s)` -- drop TRAILING blanks, and blanks only. A NUL byte is not a
// blank and Fortran does not strip it; `strlen` would stop at one, which is
// why every length is carried explicitly.
std::string_view ftrim(const char* s, int len) {
    const std::string_view v(s, static_cast<std::size_t>(len));
    return v.substr(0, v.find_last_not_of(' ') + 1);
}

// `ErrVar%ErrMsg = <expr>` REALLOCATES to exactly LEN of the right-hand side.
// Copied from parseinput_dbl_opt.cpp with this unit's name in the diagnostics
// -- including the `memset` to NUL past the new length, which unit #29
// measured in both directions.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: ParseInput_Int_Opt: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: ParseInput_Int_Opt: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// LIST-DIRECTED INPUT OF ONE INTEGER(4) FROM AN INTERNAL FILE. Everything from
// `enum class Sep` down to the end of `list_read_ints` is COPIED BYTE FOR BYTE
// from `translations/ROSCO_Helpers/parseinary_opt.cpp` (P4), which measured
// every one of its rules against gfortran's own runtime over 113 records
// (`evidence/ParseInAry_Opt/parser_conformance.{f90,cpp,txt}`). The item TYPE
// is identical -- `INTEGER(IntKi)` is `INTEGER(4)` -- so nothing about the
// reader is re-derived here.
//
// NOTHING FROM `parseinput_dbl_opt.cpp`'s READER IS USED, and that is the
// point of the split. Unit #54's separator set puts ';' in the characters that
// end a value; the INTEGER reader does not ('1;2' is 5010 with nothing stored),
// and unit #55 measured that difference over thirty-one semicolon records after
// copying the wrong set first. The REAL reader also has a store-then-fail path
// for a field with a decimal point and no digits, which the INTEGER reader does
// not have at all.
//
// TWO THINGS THAT DIFFER FROM UNIT #55, and both are the caller's rather than
// the reader's:
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
// one, and it is the same reason unit #56 needed none.
//
// AND ONE FACT ABOUT THE BUFFER THIS READER IS HANDED, which decides what
// `--sanitize` can and cannot see. `Words` is `CHARACTER(MaxParamLength) ::
// Words(2)` -- ONE contiguous 400-byte object -- so byte 201 of the record IS
// `Words(2)(1:1)`, the first character of the parameter name `FindLine` just
// matched. A mutant that reads `rec[200]` is not reading past an allocation; it
// is reading a letter. Unit #56 measured that the same option bought 5 kills
// there and 41 on `ParseInAry_Opt`, whose record IS a whole allocation.
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
// reference's behaviour and is what makes `Variable` an output on the failure
// path too. The return value is gfortran's own iostat: 0, 5010 or -1 (end of
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
// gfortran's LIST-DIRECTED OUTPUT, for the one record this unit writes.
//
// `field` and `list_directed_int` are copied from `parseinary_opt.cpp` (P4),
// which MEASURED them for an INTEGER(4) item
// (`evidence/ParseInAry_Opt/fortran_io_probe.{f90,txt}`): each INTEGER(4) is a
// self-contained 12-byte field, right-justified, and the field never overflows
// because the widest value of the type, `-2147483648`, is 11 characters.
//
// THE RECORD THIS UNIT WRITES, ROSCO_Helpers.f90:162 (clean baseline 54dd134):
//
//   PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//
//            " in input file.  Using default value of ", Variable
//
// Two items: a CHARACTER expression and ONE INTEGER(4), with NOTHING after it.
// That layout is neither sibling's -- unit #55 writes CHARACTER, an array of
// integers, then a CHARACTER, and unit #56 writes CHARACTER then one REAL -- so
// it is DERIVED from unit #55's measured grammar rules (one leading blank
// starts the record; a CHARACTER item is written raw; there is NO separator
// between a CHARACTER item and an INTEGER that follows it; an INTEGER is a
// self-contained 12-byte field with no trailing blanks) and then PRICED against
// gfortran's own record before the corpus was generated:
// `evidence/ParseInput_Int_Opt/record_form_probe.{f90,cpp,txt}`.
//
// It is COMPARED on every case that reaches the arm: `harness/ranges.toml`
// carries `vit_record = { compare_record = ... }` for this unit, so the
// reference's own record is the oracle rather than this derivation.
//
// AND ONE FACT ABOUT WHAT CAN REACH IT. `Variable = 0` is the statement
// IMMEDIATELY ABOVE the PRINT and is unconditional, so the only value this
// statement can ever write is 0 -- which is a property of the PROGRAM and not
// of the corpus. The 12-byte field is therefore always `'           0'` and no
// input moves it; the evidence README says which mutants that costs and how
// they are declared.
// ---------------------------------------------------------------------------

// Right-justify `text` in a field of `w`. Copied from parseinary_opt.cpp (P4),
// including the DELETION of gfortran's '*' overflow fill: the only caller is
// `list_directed_int`, whose `w` is 12 and whose `text` is
// `snprintf("%d", int32_t)`, at most the eleven characters of `-2147483648`, so
// the fill is a branch no input can take either way -- a mutant no input can
// kill. This campaign deletes a restatement rather than declaring its mutant
// equivalent (unit #32's `LEN_TRIM`, unit #55's `negate_cond 9381bdef`).
std::string field(const std::string& text, int w) {
    return std::string(static_cast<std::size_t>(w) - text.size(), ' ') + text;
}

// One list-directed field for an INTEGER(4).
std::string list_directed_int(int32_t v) {
    char tmp[32];
    std::snprintf(tmp, sizeof tmp, "%d", static_cast<int>(v));
    return field(std::string(tmp), 12);
}

// `PRINT *, "ROSCO Warning: Did not find "//TRIM(VarName)//" in input file.  "
//           //"Using default value of ", Variable` on unit 6.
void print_default_warning(std::string_view VarName, int32_t Variable) {
    std::string rec(1, ' ');
    rec += "ROSCO Warning: Did not find ";
    rec.append(VarName);
    rec += " in input file.  Using default value of ";
    // NO separator between a CHARACTER item and an INTEGER that follows it --
    // unit #55's measured rule. The 12-byte field carries its own leading
    // blanks, and nothing follows it, so the record ends with the value.
    rec += list_directed_int(Variable);
    rec += '\n';
    std::fwrite(rec.data(), 1, rec.size(), stdout);
}

}  // namespace

void ParseInput_Int_Opt(char* FileLines, int n_FileLines, int len_FileLines,
                        char* VarName, int len_VarName,
                        int* Variable,
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
    // unit and `ParseInput_Dbl_Opt` differ structurally from `ParseInAry_Opt`,
    // whose `GetWords` call is inside `IF (FoundLine)`. When `FoundLine` is
    // false the reference passes a `Line` that `FindLine` never assigned;
    // `GetWords` blank-fills all `NumWords` elements first, so `Words` is
    // DEFINED either way, and every read of `Words` below is inside
    // `IF (... .AND. FoundLine)` where `Line` was assigned. Reproduce the call
    // rather than routing around it (X1).
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

        // RED TEST: the PRINT deleted. Everything else, including
        // `Variable = 0` on the line above, is untouched -- so the ONLY output
        // this stub can move is the record, which is the cell `vit_record =
        // { compare_record = ... }` gave the harness an oracle in.
        (void)print_default_warning;
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
        int32_t value = static_cast<int32_t>(*Variable);
        const int ErrStatLcl = list_read_ints(Words1, MaxParamLength, &value, 1);
        *Variable = static_cast<int>(value);

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
            msg += "\" was not assigned valid INTEGER value on line #";
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
    // FOURTH of the five units carrying the identical statement. `UnEc` is a
    // Fortran UNIT NUMBER, and the record has to go to whatever file the CALLER
    // connected it to -- `<RootName>.RO.echo`, which
    // `ReadControlParameterFileSub` OPENs on it when `CntrPar%Echo > 0`
    // (ReadSetParameters.f90:358-362, clean). C++ has no access to the Fortran
    // runtime's unit table, so the only record this side could write is one to
    // `fort.<UnEc>` -- a DIFFERENT file. Writing to the wrong file is worse than
    // not writing, so nothing is emitted and the gap is named.
    //
    // THE ARM IS DEAD IN EVERY CONFIGURATION THIS CAMPAIGN TESTS, measured
    // rather than assumed and measured on THIS unit's own lines:
    // `coverage/line_coverage.json` records ROSCO_Helpers.f90:186
    // (`IF (PRESENT(UnEc))`) 1,680 times over the 27 scenarios and :187
    // (`IF (UnEc > 0) WRITE`) 1,624 -- so `UnEc` is PRESENT at 1,624 of the
    // 1,680 calls and greater than zero at none of them. All 22
    // `Examples/DISCON*.IN` set `Echo` to 0.
    //
    // Emitting NOTHING rather than a guarded no-op is deliberate: a translated
    // `if (UnEc > 0) { }` would be a mutable comparison no input could kill.
    // `harness/ranges.toml` holds `UnEc` at 0 for the same reason.
    (void)has_UnEc;
    (void)UnEc;
    (void)Tab;
}
