// VIT Translation Scaffold
// Function: GetWords
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE GetWords(Line, Words, NumWords)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 9407ffe5caaf
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T16:17:14Z

#include <algorithm>

// Line and Words arrive as CHARACTER(KIND=C_CHAR) arrays. Neither is
// NUL-terminated, so the length parameters are the only statement of their
// extents.
//
// `Words` IS AN ARRAY OF STRINGS, and it is the first one in this campaign.
// The Fortran dummy is `CHARACTER(*), INTENT(OUT) :: Words(NumWords)` -- two
// extents where every earlier unit's CHARACTER dummy had one: len_Words is the
// width of ONE element and NumWords is how many elements there are. The
// generated wrapper stages it column-major, exactly as Fortran lays it out:
//
//     Words_c((j - 1) * LEN(Words) + i) = Words(j)(i:i)
//
// so element j occupies the len_Words bytes at (j-1)*len_Words. That formula is
// written ONCE, in `word` below, rather than at each of the two sites that
// address an element -- unit #1's "name a size once", applied to a stride.

// A Fortran CHARACTER assignment `dst = src` is not a copy. It transfers
// min(LEN(src), LEN(dst)) bytes and BLANK-FILLS the remainder of dst; when src
// is the longer it truncates silently rather than erroring. Both assignments
// GetWords makes -- the blank prefill and the word itself -- are such an
// assignment, so the rule is written once here.
//
// BOTH LOOPS ARE 1-BASED, and that is a measurement carried from unit #6, not a
// style choice: written 0-based, BOTH boundary mutants survived there (score
// 0.882, mutation/GetPath.survivors_0based.json), because `<` -> `<=` writes a
// byte that is either immediately overwritten or one past the buffer -- neither
// a wrong ANSWER, so no value comparison can see it. 1-based, the same mutant
// leaves a byte UNWRITTEN and the comparison sees the incoming byte survive.
static void char_assign(char* dst, int len_dst, const char* src, int len_src) {
    const int n = std::min(len_src, len_dst);
    for (int i = 1; i <= n; ++i) {
        dst[i - 1] = src[i - 1];
    }
    for (int i = n + 1; i <= len_dst; ++i) {
        dst[i - 1] = ' ';
    }
}

// `Words(IW)`. Column-major, and the ONE site that knows the stride.
static char* word(char* Words, int len_Words, int IW) {
    return &Words[(IW - 1) * len_Words];
}

// The character SET of `SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )`. Seven characters:
// blank, comma, exclamation, semicolon, APOSTROPHE (the doubled `''` inside a
// single-quoted Fortran literal is one apostrophe, not two), double quote, and
// `Tab = CHAR(9)`.
//
// An ARRAY with its own size rather than a string literal with the length 7
// written beside it -- the same reasoning as GetRoot's `Separators`. A C string
// literal would also carry a NUL this Fortran set does not have.
static const char Whitespace[] = { ' ', ',', '!', ';', '\'', '"', '\t' };

// SCAN(string, set): the 1-based position of the FIRST character of `string`
// that appears in `set`, or 0 when none does. No BACK, so this is the forward
// search. The three answers are load-bearing and the reference branches on all
// three -- `> 1` is a word, `== 1` is a separator to skip, `== 0` is the end.
//
// A zero-length `string` yields 0, which is how `Line(Ch+1:)` with Ch == LEN
// reaches the `ELSE ... EXIT` branch.
static int scan_first(const char* s, int len, const char* set, int nset) {
    for (int i = 1; i <= len; ++i) {
        for (int k = 1; k <= nset; ++k) {
            if (s[i - 1] == set[k - 1]) {
                return i;
            }
        }
    }
    return 0;
}

// THERE IS NO len_trim HERE, AND THAT IS A DEPARTURE FROM LITERAL TRANSCRIPTION
// WITH A PROOF. The reference writes `IF ( LEN_TRIM( Line ) == 0 ) RETURN`.
//
// Claim: this predicate is the whole of what that statement reads. LEN_TRIM(x)
// is defined as the position of the LAST non-blank character of x, and 0 when x
// has none; so `LEN_TRIM(x) == 0` is true exactly when x contains no non-blank
// character, which is what `all_blank` returns. The VALUE of LEN_TRIM is not
// read here or anywhere else in the procedure -- the parse below scans from
// Ch = 0 over the full declared length and stops on the separator set, never on
// a trimmed bound.
//
// WHY REMOVE IT rather than transcribe and declare the survivors equivalent.
// This campaign has now measured the same shape on this same intrinsic four
// times: unit #1 (a size), unit #4 (`Conv2UC`'s loop bound, 0.696 -> 1.000),
// unit #6 (`GetPath`), unit #7 (`GetRoot`, six survivors of eight, three of them
// OUT-OF-BOUNDS READS that cannot honestly be declared equivalent). A helper
// that computes a POSITION when only its zero-ness is read has mutable sites
// that no input can make change an output; the fix is to delete the restatement
// and write the proof, not to declare the blindness away.
//
// AND THERE IS NO LOOP INDEX EITHER, for the same reason one step further in.
// Written as `for (int i = 1; i <= len; ++i) if (s[i - 1] != ' ')` this scored
// FOUR SURVIVORS, all of them mutants of that one index expression -- `s[i+1]`,
// `s[1-i]`, `s[i-2]`, and the bound `i < len`. Three are observable ONLY through
// a read outside the buffer, and unit #7 settled what to do with those: a mutant
// whose behaviour is undefined cannot honestly be declared equivalent, it can
// only be deleted along with the site that admits it. `std::all_of` states the
// same predicate with no index arithmetic to admit them, and its one remaining
// site -- `s + len` -> `s - len` -- makes last precede first and dies.
//
// The fourth is genuinely equivalent and is worth writing down, because it is a
// property of the REFERENCE and not of this transcription: `i < len` differs
// from `i <= len` only on a Line whose sole non-blank character is the LAST one,
// and on such a Line the parse below emits nothing anyway -- a word is written
// only when a separator FOLLOWS it, and there is nothing after the last
// character. So both answers produce an all-blank `Words` and no input can tell
// them apart. The guard is an optimisation of a path the loop already computes
// correctly, and only a wrongly-TRUE guard can change the output at all.
static bool all_blank(const char* s, int len) {
    return std::all_of(s, s + len, [](char c) { return c == ' '; });
}

void GetWords(char* Line, int len_Line, char* Words, int len_Words, int NumWords) {

    // Let's prefill the array with blanks.
    //
    //     DO IW=1,NumWords
    //         Words(IW) = ' '
    //     END DO
    //
    // A one-character source into a len_Words-wide destination, so char_assign
    // writes the blank and blank-fills the rest: every byte of every element.
    for (int IW = 1; IW <= NumWords; ++IW) {
        char_assign(word(Words, len_Words, IW), len_Words, " ", 1);
    }

    // Let's make sure we have text on this line.
    if (all_blank(Line, len_Line)) {
        return;
    }

    // Parse words separated by any combination of spaces, tabs, commas,
    // semicolons, single quotes, and double quotes ("whitespace").
    int Ch = 0;
    int IW = 0;

    for (;;) {

        // NextWhite = SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )
        //
        // `Line(Ch+1:)` is the tail from 1-based position Ch+1, which is the
        // pointer Line + Ch, and its length is what remains: len_Line - Ch.
        const int NextWhite = scan_first(Line + Ch, len_Line - Ch,
                                         Whitespace, (int)sizeof Whitespace);

        if (NextWhite > 1) {

            IW = IW + 1;
            // Words(IW) = Line(Ch+1:Ch+NextWhite-1) -- the NextWhite-1 bytes
            // before the separator. char_assign truncates a word wider than an
            // element and blank-fills a narrower one, which is what the Fortran
            // assignment does.
            char_assign(word(Words, len_Words, IW), len_Words,
                        Line + Ch, NextWhite - 1);

            if (IW == NumWords) {
                break;
            }

            Ch = Ch + NextWhite;

        } else if (NextWhite == 1) {

            Ch = Ch + 1;

            continue;

        } else {

            break;

        }

    }

}
