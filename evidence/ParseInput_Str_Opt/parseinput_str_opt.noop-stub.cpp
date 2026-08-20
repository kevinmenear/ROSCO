// VIT Translation Scaffold
// Function: ParseInput_Str_Opt
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE ParseInput_Str_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: f7c3939bb563
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-20T05:26:26Z

#include "vit_types.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// THIS UNIT IS `ParseInput_Int_Opt` (#57) WITH ONE DECLARATION CHANGED, AND THE
// CHANGE DELETES THE ENTIRE NUMBER READER RATHER THAN SWAPPING IT.
//
// `diff` of the two clean bodies at ROSCO_Helpers.f90:115-192 and :278-354
// (baseline 54dd134) is: the routine name, the declaration of `Variable`
// (`CHARACTER(*)` here, `INTEGER(IntKi)` there), the default value and its
// comment, `TRIM(Variable)` in the PRINT, the FORM of the READ, and
// whitespace/comments. Nothing else -- not the control flow, not the callee
// calls, not the arms, not the error message (which still says "INTEGER"; that
// is the reference's own text and P7 makes the source the oracle, not the
// sense).
//
// So the CALLER half -- constants, `ftrim`, `assign_errmsg`, `int2lstr_trimmed`
// and the whole subroutine body -- is copied from
// `translations/ROSCO_Helpers/parseinput_int_opt.cpp` (P4). The block hashes
// are in `evidence/ParseInput_Str_Opt/p4_blocks.txt`, written by
// `check_p4_blocks.py`, which re-extracts each block from its source file and
// compares bytes.
//
// AND THE ITEM HALF IS THIS UNIT'S OWN, because there is no sibling to take it
// from: no unit in this campaign has read a CHARACTER item before.
//
//     READ (Words(1),*   ,IOSTAT=ErrStatLcl) Variable     <- #55, #56, #57
//     READ (Words(1),'(A)',IOSTAT=ErrStatLcl) Variable    <- HERE
//
// The second is not list-directed input at all. It is ONE `A` edit descriptor
// with no field width, applied to a `CHARACTER(len_Variable)` item, over an
// internal file made from a `CHARACTER(200)` scalar -- one record, 200 bytes,
// blank-padded. Fortran 2018 13.7.4.1: an `A` descriptor with no `w` takes
// `w = LEN(item)`; 12.6.4.5.3: on input a short record is blank-padded when
// `PAD='YES'`, which is the default and which an internal file cannot override
// here. `w == LEN(item)` means the field is transferred whole. So the statement
// is `Variable = Words(1)` under Fortran's own CHARACTER-assignment rule --
// truncate on the right if the item is shorter, blank-pad on the right if it is
// longer -- and NOTHING IN IT CAN FAIL.
//
// That last clause is a claim about the PROGRAM, so it is MEASURED rather than
// argued: `evidence/ParseInput_Str_Opt/record_form_probe.{f90,cpp,txt}` runs
// the reference's own READ over every record form this unit's corpus can
// contain, crossed with the item lengths its shipped callers use and the two
// boundaries at 200, and reports (IOSTAT, bytes) from both sides.
//
// THE CONSEQUENCE, stated here rather than left to be found: `ErrStatLcl` is 0
// on every input, so `IF (ErrStatLcl /= 0)` and its whole message block are
// dead IN THE PROGRAM. They are translated anyway (P7: mirror the source, and
// X1: `Int2LStr` is called rather than inlined), and the mutants that land in
// them are declared `unreachable` from this translation's own line coverage --
// the same mechanism unit #57 used for the comma and semicolon branches of its
// reader, and for the same kind of reason: the cause is the PROGRAM, not the
// corpus, so no corpus change empties the group.
//
// AND ONE THING A NAIVE COPY WOULD GET WRONG, which is why the 200-byte bound
// is carried explicitly. `Words` is `CHARACTER(MaxParamLength) :: Words(2)` --
// ONE contiguous 400-byte object -- so byte 201 of that object is `Words(2)`,
// the parameter NAME `FindLine` just matched. Every shipped caller passes a
// `Variable` of length 1024 or 256, both greater than 200. A `memcpy` of
// `len_Variable` bytes from `Words.data()` would therefore splice the parameter
// name into the value on EVERY shipped call. The internal file is `Words(1)`
// alone: 200 bytes and then blanks.
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
// `GetWords`'s bridge takes as `len_Words` -- AND the RECORD LENGTH of the
// internal-file READ: the reference reads from `Words(1)`, not from `Line`.
constexpr int MaxParamLength = 200;

// `Int2LStr`'s result is `CHARACTER(11)` (ROSCO_Helpers.f90:1610). The bridge
// writes all 11 bytes, blank-padded, and does not NUL-terminate.
constexpr int Int2LStrLen = 11;

// `CHARACTER(*), PARAMETER :: RoutineName = 'ParseInput_Str_Opt'`.
constexpr std::string_view RoutineName = "ParseInput_Str_Opt";

// `Variable = 'unused'     ! Default of string input is unused for now`.
// A CHARACTER literal, so the assignment truncates it when the caller's item is
// shorter than six -- which is what makes `default_value` a function of
// `len_Variable` rather than a constant, and is the one way this unit's PRINT
// record can carry something other than "unused".
constexpr std::string_view DefaultValue = "unused";

// ROSCO_Helpers.f90:25 -- `LOGICAL, PARAMETER :: DEBUG_PARSING = .FALSE.`.
// A PARAMETER, so the debug block guarded by it is dead in every build of this
// tree; it is not translated and the evidence README says so. What IS
// translated is the `GetWords` call ABOVE the block, which the reference makes
// unconditionally.
constexpr bool DEBUG_PARSING = false;

// ---------------------------------------------------------------------------
// String helpers -- copied from parseinput_int_opt.cpp (P4), same module.
// ---------------------------------------------------------------------------

// `TRIM(s)` -- drop TRAILING blanks, and blanks only. A NUL byte is not a
// blank and Fortran does not strip it; `strlen` would stop at one, which is
// why every length is carried explicitly.
std::string_view ftrim(const char* s, int len) {
    const std::string_view v(s, static_cast<std::size_t>(len));
    return v.substr(0, v.find_last_not_of(' ') + 1);
}

// `ErrVar%ErrMsg = <expr>` REALLOCATES to exactly LEN of the right-hand side.
// Copied from parseinput_int_opt.cpp with this unit's name in the diagnostics
// -- including the `memset` to NUL past the new length, which unit #29
// measured in both directions.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: ParseInput_Str_Opt: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: ParseInput_Str_Opt: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// THE ITEM HALF -- this unit's own, and the whole of it.
// ---------------------------------------------------------------------------

// A Fortran CHARACTER assignment `dst = src`: transfer min(LEN(src), LEN(dst))
// bytes and BLANK-FILL the remainder of dst, truncating silently when src is
// the longer. Fortran 2018 10.2.1.3(10). This is the ONE rule both statements
// below need, so it is written once:
//
//     Variable = 'unused'                    <- src is the 6-byte literal
//     READ (Words(1),'(A)') Variable         <- src is the 200-byte record
//
// and the second is that statement, for the reason set out at the top of the
// file.
//
// COPIED BYTE FOR BYTE from `translations/ROSCO_Helpers/findline.cpp` (P4,
// unit #32), and the reason to copy THAT form rather than write the obvious
// one is a MEASUREMENT that unit already paid for. The obvious form is
//
//     const int n = std::min(len_dst, len_src);
//     if (n > 0)        std::memcpy(dst, src, n);
//     if (len_dst > n)  std::memset(dst + n, ' ', len_dst - n);
//
// and it was this unit's first draft. Its first sweep left THREE of its
// mutants alive, which are the three unit #32's comment names at `GetWords`'
// version of the same expression:
//
//     8b83f1b6  min(len_dst, len_src) -> min(len_src, len_dst)   SURVIVED
//     dac6e85c  'if (n > 0)'          -> 'if (n >= 0)'           SURVIVED
//     1db4dd8f  'if (len_dst > n)'    -> 'if (len_dst >= n)'     SURVIVED
//
// -- `min` is commutative, and a `memcpy`/`memset` of zero bytes is a no-op, so
// all three are behaviour-preserving and could only ever be DECLARED. Unit #7's
// rule settles what to do with a mutant no value comparison can reach: delete
// the SITE that admits it rather than argue about it. The loop below has no
// `min` to swap and no zero-length guard to widen; its protection is the loop's
// own bound, and its one surviving predicate, `i <= len_src`, changes an ANSWER
// at exactly the truncation boundary -- which this unit's corpus straddles,
// because `Variable = 'unused'` truncates at every `len_Variable < 6` and pads
// at every one above it (808 cases of 14,116 on the first side,
// `evidence/ParseInput_Str_Opt/harness_partition.txt`).
//
// It is also the safer program: `dst` cannot be left by a loop that counts to
// `len_dst`.
void char_assign(char* dst, int len_dst, const char* src, int len_src) {
    for (int i = 1; i <= len_dst; ++i) {
        dst[i - 1] = (i <= len_src) ? src[i - 1] : ' ';
    }
}

// `READ (Words(1),'(A)',IOSTAT=ErrStatLcl) Variable`, returning gfortran's own
// IOSTAT. It is 0 on every input -- see the file header and
// `evidence/ParseInput_Str_Opt/record_form_probe.txt`, which measures it
// against the reference rather than asserting it -- and the value is returned
// rather than assumed at the call site because the reference TESTS it, and a
// model of the test is not a model of the READ.
int read_a_edit(const char* rec, int len_rec, char* item, int len_item) {
    char_assign(item, len_item, rec, len_rec);
    return 0;
}

// ---------------------------------------------------------------------------
// THE PRINT RECORD
//
// `PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input file.
//           Using default value of ", TRIM(Variable)`
//
// TWO CHARACTER ITEMS, which is a layout no sibling in this family has: unit
// #55 writes CHARACTER, an array of INTEGERs, then a CHARACTER; #56 writes
// CHARACTER then one REAL; #57 writes CHARACTER then one INTEGER. So the rule
// that decides this record is the one none of them exercised -- what
// libgfortran puts BETWEEN two adjacent CHARACTER items -- and it is the whole
// content of the derivation:
//
//     list_formatted_write_scalar, libgfortran/io/write.c:
//         if (dtp->u.p.first_item) { ...; write_char (dtp, ' '); }
//         else if (type != BT_CHARACTER || !dtp->u.p.char_flag
//                  || dtp->u.p.current_unit->delim_status != DELIM_NONE)
//             write_separator (dtp);
//
// `delim_status` is DELIM_NONE for list-directed output to an external unit
// unless DELIM= says otherwise, and `char_flag` is set by the PRECEDING item.
// A CHARACTER after a CHARACTER therefore takes NO separator at all -- the two
// items abut. The leading blank is the record's, written once for the first
// item, which is unit #55's measured rule and is unchanged.
//
// PRICED AGAINST GFORTRAN BEFORE THE CORPUS WAS GENERATED, because a rule read
// off a source file is still a derivation:
// `evidence/ParseInput_Str_Opt/print_record_probe.{f90,cpp,txt}`.
//
// AND IT IS COMPARED ON EVERY CASE THAT REACHES THE ARM: `harness/ranges.toml`
// carries `vit_record = { compare_record = ... }` for this unit, so the
// reference's own record is the oracle rather than this derivation.
//
// WHAT CAN REACH IT. `Variable = 'unused'` is the statement immediately above
// the PRINT and is unconditional, so the value item is `TRIM('unused')`
// truncated to `len_Variable` -- "unused" for every shipped caller (1024 and
// 256) and something shorter only where the corpus makes `len_Variable < 6`.
// That is the one input this unit's record has, and the corpus is what supplies
// it.
// ---------------------------------------------------------------------------
void print_default_warning(std::string_view VarName, std::string_view Variable) {
    std::string rec(1, ' ');
    rec += "ROSCO Warning: Did not find ";
    rec.append(VarName);
    rec += " in input file.  Using default value of ";
    // NO separator between two CHARACTER items -- the rule above.
    rec.append(Variable);
    rec += '\n';
    std::fwrite(rec.data(), 1, rec.size(), stdout);
}

}  // namespace

void ParseInput_Str_Opt(char* FileLines, int n_FileLines, int len_FileLines,
                        char* VarName, int len_VarName,
                        char* Variable, int len_Variable,
                        char* FileName, int len_FileName,
                        errorvariables_view_t* ErrVar,
                        int has_AllowDefault, int32_t AllowDefault,
                        int has_UnEc, int UnEc) {
    // NO-OP STUB -- the whole body deleted. The red test for the differential
    // harness itself: whatever this moves is what the harness can see at all.

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

    (void)Variable; (void)len_Variable; (void)FileName; (void)len_FileName;
    (void)ErrVar; (void)has_AllowDefault; (void)AllowDefault;
    (void)has_UnEc; (void)UnEc;
    (void)ftrim; (void)assign_errmsg; (void)int2lstr_trimmed;
    (void)char_assign; (void)read_a_edit; (void)print_default_warning;
    (void)DEBUG_PARSING; (void)NewLine; (void)Tab; (void)MaxLineLength;
    (void)MaxParamLength; (void)RoutineName; (void)DefaultValue;
}
