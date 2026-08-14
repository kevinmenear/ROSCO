// VIT Translation Scaffold
// Function: FindLine
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 3c83f8c7c74c
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-14T16:14:56Z

#include <algorithm>
#include <cstdint>
#include <vector>

namespace {

// The two CHARACTER widths this unit depends on are module PARAMETERs of
// ROSCO_Helpers.f90 (lines 44-45 of the clean source), not arguments:
//
//     INTEGER(IntKi), PARAMETER :: MaxLineLength  = 2048
//     INTEGER(IntKi), PARAMETER :: MaxParamLength = 200
//
// NEITHER CROSSES THE SIGNATURE, and that is worth stating rather than assuming.
// `Line` is declared `CHARACTER(MaxLineLength), INTENT(OUT)`, so VIT resolved
// its length at scaffold time and emitted no `len_Line` parameter -- the
// constant below is the ONLY statement of that buffer's extent on the C++ side,
// and it has to agree with the staging buffer the generated Fortran wrapper
// declares. `MaxParamLength` is the element width of the local `Words` array and
// of the two uppercase locals; it never leaves this function.
//
// Each is named once, here, and read wherever the width is needed (unit #1).
constexpr int MaxLineLength = 2048;
constexpr int MaxParamLength = 200;

// A Fortran CHARACTER assignment `dst = src` transfers min(LEN(src), LEN(dst))
// bytes and BLANK-FILLS the remainder of dst; when src is the longer it
// truncates silently. Every assignment this unit makes is of that kind -- three
// of them -- so the rule is written once. Carried from GetWords (unit #8) with
// its 1-based loops intact: written 0-based, both boundary mutants survive
// there, because `<` -> `<=` writes a byte that is either overwritten at once or
// one past the buffer, and neither is a wrong ANSWER.
void char_assign(char* dst, int len_dst, const char* src, int len_src) {
    const int n = std::min(len_src, len_dst);
    for (int i = 1; i <= n; ++i) {
        dst[i - 1] = src[i - 1];
    }
    for (int i = n + 1; i <= len_dst; ++i) {
        dst[i - 1] = ' ';
    }
}

// `Words(IW)` and `FileLines(I)`, which are the same shape: a rank-1 CHARACTER
// array is laid out element-major with a stride of one element's width, and the
// generated wrapper stages both exactly that way --
//
//     X_c((j - 1) * LEN(X) + i) = X(j)(i:i)
//
// One site knows the stride, addressed from three places (unit #8's `word`).
char* element(char* base, int width, int j) {
    return &base[(j - 1) * width];
}

}  // namespace

// FileLines, ParamName and Line arrive as CHARACTER(KIND=C_CHAR) arrays. None is
// NUL-terminated: `n_FileLines` and `len_FileLines` are the only statement of the
// input array's two extents, `len_ParamName` of the name's, and `MaxLineLength`
// above of `Line`'s.
//
// `AryLen` IS OPTIONAL IN THE FORTRAN AND CROSSES AS TWO ARGUMENTS -- a presence
// flag and a value. That is P6 in the signature: absence has its own
// representation and does not have to be encoded as some value of AryLen. Both
// arms are live in this campaign's own inputs, measured on
// coverage/line_coverage.json rather than argued: of 5,354 calls in the 27
// scenarios, 3,892 take `.NOT. PRESENT` (clean ROSCO_Helpers.f90:1097, the three
// ParseInput_* call sites) and 1,462 take the other (line 1099, the two ParseAry
// call sites).
//
// WHAT THE REFERENCE DOES FOR AryLen <= -1 IS UNDEFINED, and this translation
// does not invent an answer for it. `ALLOCATE(Words(WordInd))` with WordInd <= 0
// yields a ZERO-SIZED array in Fortran; `GetWords` then either writes `Words(1)`
// (a non-blank line) or reads nothing, and `FileLineUC = Words(WordInd)` reads
// `Words(0)` or below. Those are reads and writes outside the allocation, so
// there is no reference behaviour to mirror -- see evidence/FindLine/README.md
// and the `arylen` entry in harness/ranges.toml, which holds the corpus at
// AryLen >= 0. AryLen == 0 is INSIDE the defined domain (WordInd == 1) and is
// exercised, so the boundary itself is not excluded, only the region past it.
// Both call sites that pass AryLen guard `IF (AryLen < 1)` immediately AFTER
// this call, so the caller does not exclude it either.
void FindLine(char* FileLines, int n_FileLines, int len_FileLines, char* ParamName, int len_ParamName, int32_t* FoundLine, char* Line, int* LineNum, int has_AryLen, int AryLen) {

    // IF (.NOT. PRESENT(AryLen)) THEN
    //     WordInd = 2
    // ELSE
    //     WordInd = AryLen + 1
    // ENDIF
    //
    // WordInd is the index of the word that carries the NAME. A scalar
    // parameter's line is `<value> <name>`, so the name is word 2; an array
    // parameter's line is `<v1> ... <vAryLen> <name>`, so it is word AryLen + 1.
    int WordInd;
    if (!has_AryLen) {
        WordInd = 2;
    } else {
        WordInd = AryLen + 1;
    }

    // ALLOCATE(Words(WordInd))   ! TODO: check error
    //
    // WordInd elements of MaxParamLength bytes each, laid out as the Fortran
    // lays them out so that `element` addresses them and `getwords_c` -- whose
    // Fortran side declares `CHARACTER(*) :: Words(NumWords)` -- writes them.
    // The reference's own comment marks the unchecked STAT; there is nothing to
    // mirror, since a failed ALLOCATE would abort both sides alike.
    std::vector<char> Words(static_cast<std::size_t>(MaxParamLength) *
                            static_cast<std::size_t>(std::max(WordInd, 0)));

    // ParamNameUC = ParamName
    // CALL Conv2UC(ParamNameUC)
    //
    // The local is CHARACTER(MaxParamLength), so a name longer than 200
    // characters is compared TRUNCATED -- the assignment below is where that
    // happens, and it happens to the search key rather than to the file line.
    // Conv2UC runs over the whole declared width, which is what the reference
    // passes it: the dummy is CHARACTER(*) and the actual argument is the local,
    // not a trimmed section of it.
    char ParamNameUC[MaxParamLength];
    char_assign(ParamNameUC, MaxParamLength, ParamName, len_ParamName);
    // NO-UPPERCASE STUB: `CALL Conv2UC(ParamNameUC)` removed

    // Declared once outside the loop because the reference declares it once
    // outside the loop. Nothing carries across an iteration -- the assignment
    // below is a full-width CHARACTER assignment and overwrites all 200 bytes --
    // but the scope is the reference's and is transcribed rather than improved.
    char FileLineUC[MaxParamLength];

    // Search for line in FileLines
    *FoundLine = 0;   // FoundLine = .FALSE.
    *LineNum = 0;

    // THE LOOP DOES NOT EXIT ON A MATCH, and that is the unit's one piece of
    // behaviour that a casual reading inverts: when a name appears on more than
    // one line of the file, the LAST occurrence wins, because each match
    // overwrites Line and LineNum. Transcribed as written.
    for (int I = 1; I <= n_FileLines; ++I) {

        char* FileLine = element(FileLines, len_FileLines, I);

        // CALL GetWords ( FileLines(I), Words, WordInd )
        //
        // Through the translated callee's bridge, never re-derived here (X1).
        // GetWords blank-fills all WordInd elements before parsing, so `Words`
        // needs no initialisation of its own and carries nothing across
        // iterations.
        getwords_c(FileLine, len_FileLines, Words.data(), MaxParamLength, WordInd);

        // FileLineUC = Words(WordInd)
        // CALL Conv2UC(FileLineUC)
        //
        // Source and destination are both CHARACTER(MaxParamLength), so this
        // assignment neither truncates nor pads; char_assign is used anyway so
        // that the width is read from the one place that states it.
        char_assign(FileLineUC, MaxParamLength,
                    element(Words.data(), MaxParamLength, WordInd), MaxParamLength);
        conv2uc_c(FileLineUC, MaxParamLength);

        // IF (FileLineUC == ParamNameUC) THEN
        //
        // THERE IS NO TRIM ON EITHER SIDE, unlike ChkParseData's comparison of
        // the same two quantities. Both operands are CHARACTER(MaxParamLength)
        // and both were produced by a blank-filling assignment, so Fortran's
        // rule -- pad the shorter operand with blanks and compare -- degenerates
        // to a byte-for-byte comparison of 200 bytes. std::equal states that
        // with no index arithmetic of its own to make mutable.
        if (std::equal(FileLineUC, FileLineUC + MaxParamLength, ParamNameUC)) {
            // Line = FileLines(I)
            //
            // Line is CHARACTER(MaxLineLength) and FileLines(I) is
            // CHARACTER(len_FileLines): a shorter file line is blank-filled out
            // to 2048, a longer one is truncated at 2048.
            char_assign(Line, MaxLineLength, FileLine, len_FileLines);
            *LineNum = I;
            *FoundLine = 1;   // FoundLine = .TRUE.
        }
    }

    // ON NO MATCH, `Line` IS LEFT EXACTLY AS IT ARRIVED. The reference declares
    // it INTENT(OUT) and assigns it only inside the IF, so its contents are
    // undefined-by-the-standard and in practice are whatever the caller's
    // storage held -- and both of the ParseInput_* callers read it afterwards
    // (`CALL GetWords ( Line, Words, 2 )`) before testing FoundLine. Writing
    // blanks here would be a defensible program and a different one; a mirror
    // contract says leave it. The generated wrapper stages `Line` IN as well as
    // OUT, so the caller's bytes are what this function is handed.
}
