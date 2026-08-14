// VIT Translation Scaffold
// Function: ChkParseData
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE ChkParseData(Words, ExpVarName, FileName, FileLineNum, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 782245bac5db
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-14T02:01:48Z

#include "vit_types.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// SysGnuLinux.f90:37 -- `CHARACTER(*), PARAMETER :: NewLine = ACHAR(10)`.
// One byte, and it is read out of the source rather than recalled: the same
// declaration exists in seven SysFiles variants and every one of them is
// ACHAR(10), so no build of this tree can make it CR or CRLF. P7 reaches a
// named constant as well as an expression (unit #28).
constexpr char NewLine = '\n';

// `CHARACTER(20) :: ExpUCVarName, FndUCVarName` -- the two locals' declared
// width, named ONCE. Both call sites in this campaign hand it words of
// `MaxParamLength = 200`, so the assignment below TRUNCATES, and the width is
// the whole of why. Unit #1's "name a size once".
constexpr int UCVarNameLen = 20;

// `Int2LStr`'s result is `CHARACTER(11)` (ROSCO_Helpers.f90:1610). The bridge
// writes all 11 bytes, blank-padded, and does not NUL-terminate.
constexpr int Int2LStrLen = 11;

// A Fortran CHARACTER assignment `dst = src` is not a memcpy. It transfers
// min(LEN(dst), LEN(src)) bytes and then BLANK-fills the remainder of dst; if
// src is longer, the tail is dropped with no diagnostic. Both directions
// happen here -- `FndUCVarName = Words(1)` truncates a 200-byte word into 20
// bytes, and `ExpUCVarName = ExpVarName` blank-pads a short name out to 20.
//
// NO GUARD ON EITHER CALL, AND THE ABSENCE IS MEASURED RATHER THAN TIDY.
// `n` is a minimum of two lengths, so `n >= 0` and `len_dst - n >= 0` hold by
// construction and a zero-length `memcpy`/`memset` is well defined on valid
// pointers -- `dst + len_dst` is a legal one-past-the-end address. A guard here
// is a restatement of a quantity nothing downstream can read, and this campaign
// has measured what that costs three times (units #1, #4, and the first sweep
// of this unit): `if (n > 0)` and `if (len_dst > n)` contributed four surviving
// mutants between them, not because they are equivalent but because no input
// can make them change an output. `std::min` rather than a written-out
// comparison for the same reason -- it has no mutable `<`.
void char_assign(char* dst, int len_dst, const char* src, int len_src) {
    const int n = std::min(len_src, len_dst);
    std::memcpy(dst, src, static_cast<std::size_t>(n));
    std::memset(dst + n, ' ', static_cast<std::size_t>(len_dst - n));
}

// `TRIM(s)` -- drop TRAILING blanks, and blanks only. A NUL byte is not a
// blank and Fortran does not strip it; `strlen` would stop at one, which is
// why the length is carried explicitly everywhere below.
// The same shape `checkinputs.cpp::errmsg_trim` ships (P4), not a hand-written
// scan. The written-out loop `while (n > 0 && s[n-1] == ' ') --n;` has three
// mutable sites where this has none, and the first sweep of this unit left two
// of them alive -- `n >= 0`, which reads `s[-1]`, and `n > 1`. `find_last_not_of`
// returns `npos` for an all-blank string and `npos + 1` is 0, so the empty case
// needs no branch either.
std::string_view ftrim(const char* s, int len) {
    const std::string_view v(s, static_cast<std::size_t>(len));
    return v.substr(0, v.find_last_not_of(' ') + 1);
}

// `Words(IW)` -- the dummy is `CHARACTER(*), INTENT(IN) :: Words(2)`, so
// `len_Words` is the width of ONE element and the two elements are laid out
// contiguously, column-major, element j at (j-1)*len_Words. The stride is
// written once, here, exactly as GetWords (unit #8) writes it.
const char* word(const char* Words, int len_Words, int IW) {
    return &Words[(IW - 1) * len_Words];
}

// `ErrVar%ErrMsg` is `CHARACTER(:), ALLOCATABLE`, so the Fortran assignment
// REALLOCATES it to exactly LEN of the right-hand side -- there are no bytes
// past the new length in the reference at all. The staging buffer the view
// crosses on is wider than that, and what belongs in the region past
// `n_ErrMsg` is decided by whoever allocated the buffer.
//
// MEASURED on unit #29, in both directions, before this shape was settled:
// leaving the previous message's tail in place failed 16,729 of 16,769 and
// blank-filling it failed 16,769 of 16,769, every failure on `ErrVar.ErrMsg`
// and none on `n_ErrMsg`. The first differing byte -- `a=0x20 b=0x00` at
// exactly index `n_ErrMsg` -- is what said which way to go: the oracle side is
// a freshly zeroed buffer the bridge writes `n_ErrMsg` bytes into, so the
// discarded region reads as NUL. Copied from `checkinputs.cpp`, not
// re-derived (P4).
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: ChkParseData: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: ChkParseData: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    // Unguarded, for the reason `char_assign` states: the refusal above has
    // already established `s.size() <= n_ErrMsg_cap`, so the count is
    // non-negative and a zero-length `memset` is well defined. The guard that
    // used to be here survived mutation twice -- once as `>=` and once negated.
    std::memset(ErrVar->ErrMsg + s.size(), 0,
                static_cast<std::size_t>(ErrVar->n_ErrMsg_cap) - s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// The two error messages differ in ONE substring -- `TRIM(Words(1))` in the
// first arm and `TRIM(ExpVarName)` in the second -- and are otherwise the same
// five literal pieces. The reference writes both out in full, twice; writing
// them twice here would put every one of those literals at two mutable sites,
// where a mutation of either copy is killed only by the cases that reach that
// arm. One name, one site (units #1 and #4).
std::string parse_error_message(std::string_view name,
                                const char* FileName, int len_FileName,
                                int FileLineNum) {
    // TRIM( Int2LStr( FileLineNum ) ) -- through the translated callee's
    // bridge, never re-derived here (X1). The buffer is caller-owned and all
    // 11 bytes come back written.
    char lineNumStr[Int2LStrLen];
    int2lstr_c(FileLineNum, lineNumStr);

    std::string out;
    out += " >> A fatal error occurred when parsing data from \"";
    out += ftrim(FileName, len_FileName);
    out += "\".";
    out += NewLine;
    out += " >> The variable \"";
    out += name;
    out += "\" was not assigned a valid value on line #";
    out += ftrim(lineNumStr, Int2LStrLen);
    out += '.';
    return out;
}

}  // namespace

// `NameIndx` IS NOT TRANSLATED, and the omission is the point rather than an
// oversight. The reference declares `INTEGER(IntKi) :: NameIndx` as a LOCAL --
// not INTENT(OUT), not SAVEd -- assigns it 1 in the first arm and 2 in the
// third, and never reads it. No output of this subroutine depends on it, so a
// transcribed copy would be two mutable sites that no input can make
// observable: they survive mutation not because they are equivalent but
// because the instrument is blind to them, and declaring that as equivalence
// would record a property of the harness as a property of the mutant. Units #1
// and #4 measured exactly that cost (Conv2UC: six such sites, score 0.696).
void ChkParseData(char* Words, int len_Words, char* ExpVarName, int len_ExpVarName, char* FileName, int len_FileName, int FileLineNum, errorvariables_view_t* ErrVar) {
    char FndUCVarName[UCVarNameLen];
    char ExpUCVarName[UCVarNameLen];

    // FndUCVarName = Words(1)
    // ExpUCVarName = ExpVarName
    char_assign(FndUCVarName, UCVarNameLen, word(Words, len_Words, 1), len_Words);
    char_assign(ExpUCVarName, UCVarNameLen, ExpVarName, len_ExpVarName);

    // CALL Conv2UC ( FndUCVarName )
    // CALL Conv2UC ( ExpUCVarName )
    conv2uc_c(FndUCVarName, UCVarNameLen);
    conv2uc_c(ExpUCVarName, UCVarNameLen);

    // IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )
    //
    // Fortran compares two CHARACTERs by blank-padding the shorter to the
    // longer's length. Both sides here have already had their trailing blanks
    // removed, so the padded comparison can only succeed when the two trimmed
    // lengths are equal -- the longer one's last byte is non-blank by the
    // definition of TRIM, and the padding supplies a blank opposite it.
    // `operator==` on two string_views is exactly that: equal size and equal
    // bytes.
    if (ftrim(FndUCVarName, UCVarNameLen) == ftrim(ExpUCVarName, UCVarNameLen)) {
        // The name was found in Words(1), which is the VALUE position -- so the
        // variable was not assigned a value. This arm is the error.
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      parse_error_message(ftrim(word(Words, len_Words, 1), len_Words),
                                          FileName, len_FileName, FileLineNum));
        return;
    } else {
        // FndUCVarName = Words(2)
        // CALL Conv2UC ( FndUCVarName )
        char_assign(FndUCVarName, UCVarNameLen, word(Words, len_Words, 2), len_Words);
        conv2uc_c(FndUCVarName, UCVarNameLen);

        if (ftrim(FndUCVarName, UCVarNameLen) == ftrim(ExpUCVarName, UCVarNameLen)) {
            // NameIndx = 2 -- see the note above the function. Nothing is
            // written to ErrVar on this path, and that includes ErrMsg: the
            // reference leaves whatever the caller had there.
        } else {
            // Neither word is the expected name.
            //
            // The message names `TRIM( ExpVarName )` here where the first arm
            // names `TRIM( Words(1) )`, and it is the UNTRUNCATED argument
            // rather than the 20-byte `ExpUCVarName` local -- so a name longer
            // than 20 characters is compared truncated and reported in full.
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          parse_error_message(ftrim(ExpVarName, len_ExpVarName),
                                              FileName, len_FileName, FileLineNum));
            return;
        }
    }
}
