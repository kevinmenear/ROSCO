// VIT Translation Scaffold
// Function: GetRoot
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE GetRoot(GivenFil, RootName)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 9d68a808d661
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T15:09:54Z

#include <algorithm>

// GivenFil and RootName arrive as CHARACTER(KIND=C_CHAR) arrays of exactly
// len_GivenFil and len_RootName bytes. Neither is NUL-terminated, so those two
// parameters are the only statement of their extents, and each is named once.
//
// At the ONE call site in this campaign the two actual arguments are THE SAME
// VARIABLE -- `CALL GetRoot(RootName,RootName)` at DISCON.F90:67. That aliasing
// is a property of the caller and this translation neither relies on it nor is
// harmed by it: every write to RootName here is immediately followed by a
// return, so no statement reads GivenFil after RootName has been written. The
// generated wrapper stages both dummies into separate C_CHAR arrays, so the
// aliasing does not reach this function at all.

// THERE IS NO len_trim HERE, AND THAT IS A DEPARTURE FROM LITERAL TRANSCRIPTION
// WITH A PROOF. The reference names LEN_TRIM(GivenFil) at two sites -- the DO
// bound and the `I <` test -- and this translation uses len_GivenFil at both.
//
// Claim: GetRoot's answer is unchanged if LEN_TRIM(GivenFil) is replaced by
// LEN(GivenFil) at both sites. Three parts, one per use.
//
//   1. `TRIM(GivenFil) == "."` and `GivenFil == "."` are the SAME PREDICATE in
//      Fortran. Comparison blank-pads the shorter operand to the length of the
//      longer, and TRIM removes only blanks, so both forms compare the same
//      bytes against the same padding. The reference's TRIM is a no-op there by
//      the language's own rule. Same for `".."`.
//   2. The scan `DO I = <bound>, 1, -1` looks for '.'. Every position between
//      LEN_TRIM and LEN is a blank, and a blank is not a '.', so the larger
//      bound finds the same I.
//   3. `I < LEN_TRIM` and `I < LEN` differ only when I == LEN_TRIM < LEN, since
//      the loop gives I <= LEN_TRIM. There the character at I+1 is a BLANK, so
//      INDEX('\/', blank) is 0 and this translation takes `GivenFil(:I-1)`. The
//      reference takes its ELSE, which is `GivenFil(:I-1)` when I /= 1 and the
//      zero-length '' when I == 1 -- and `GivenFil(:0)` IS the zero-length
//      string, so the two agree on both. Note the second half of `I == LEN_TRIM`
//      is unreachable anyway: I == 1 == LEN_TRIM means TRIM(GivenFil) is ".",
//      which part 1's special case has already returned on.
//
// WHY REMOVE IT rather than transcribe and declare the survivors equivalent.
// Unit #4 measured this exact shape on this exact intrinsic: `Conv2UC` restated
// LEN_TRIM in a loop bound and SIX mutable sites computing a quantity nothing
// downstream could read all survived. Transcribed here, a `len_trim` helper
// scored the same way -- six survivors of eight, mutation/GetRoot.survivors_len_trim.json
// -- and three of them (`s[i+1]`, `s[1-i]`, `s[i-2]`) are OUT-OF-BOUNDS READS.
// A mutant whose behaviour is undefined cannot honestly be DECLARED equivalent;
// it can only be deleted along with the site that admits it.

// Fortran CHARACTER equality is not memcmp. The shorter operand is blank-padded
// to the length of the longer and then compared byte for byte, so `"." == ".  "`
// is TRUE and a length difference is not by itself a difference. GetRoot's two
// special cases are both such a comparison, against a literal shorter than or
// equal to the trimmed name, so the rule is written once here rather than
// re-derived at each site.
static bool char_eq(const char* a, int la, const char* b, int lb) {
    const int n = std::max(la, lb);
    for (int i = 1; i <= n; ++i) {
        const char ca = (i <= la) ? a[i - 1] : ' ';
        const char cb = (i <= lb) ? b[i - 1] : ' ';
        if (ca != cb) {
            return false;
        }
    }
    return true;
}

// The character SET of `INDEX( '\/', GivenFil(I+1:I+1) )`: a directory
// separator, either kind. In Fortran a backslash is an ordinary character (this
// build does not pass -fbackslash), so `'\/'` is the two bytes 0x5C 0x2F; in C++
// the first has to be escaped to mean itself.
//
// An ARRAY with its own size rather than a string literal with the length 2
// written beside it. That is unit #1's "name a size once": as `index_first("\\/",
// 2, c)` the digit is a mutable site restating what the array already fixes, and
// its mutant reads the NUL terminator a C string literal carries and this
// Fortran set does not -- a site no input can reach, because the generator
// refuses to put a NUL in a CHARACTER argument.
static const char Separators[] = { '\\', '/' };

// INDEX(string, substring) for a ONE-CHARACTER substring: the 1-based position
// of the FIRST occurrence, or 0 when there is none. No BACK, so this is the
// forward search -- the mirror of GetPath's index_back, and deliberately a
// separate function rather than a parameterised one, because the reference
// calls two different intrinsics.
static int index_first(const char* s, int len, char c) {
    for (int i = 1; i <= len; ++i) {
        if (s[i - 1] == c) {
            return i;
        }
    }
    return 0;
}

// A Fortran CHARACTER assignment `dst = src` is not a copy. It transfers
// min(LEN(src), LEN(dst)) bytes and BLANK-FILLS the remainder of dst; when src
// is the longer it truncates silently rather than erroring. All six assignments
// to RootName in GetRoot are such an assignment, so the rule is written once
// here. Writing the fill bound at each site would restate a quantity that
// `len_dst` already fixes -- unit #1's "name a size once".
//
// BOTH LOOPS ARE 1-BASED, and that is a measurement, not a style choice. Unit
// #6 wrote the same helper 0-based and BOTH boundary mutants SURVIVED (score
// 0.882, mutation/GetPath.survivors_0based.json): `<` -> `<=` on the copy loop
// writes dst[n], which the fill loop immediately overwrites, and on the fill
// loop it writes dst[len_dst], one byte past the buffer. Neither is a wrong
// ANSWER, so no value comparison can see either. 1-based, the same mutant
// LEAVES A BYTE UNWRITTEN -- dst keeps its incoming byte, the comparison sees
// it, and both die.
static void char_assign(char* dst, int len_dst, const char* src, int len_src) {
    const int n = std::min(len_src, len_dst);
    for (int i = 1; i <= n; ++i) {
        dst[i - 1] = src[i - 1];
    }
    for (int i = n + 1; i <= len_dst; ++i) {
        dst[i - 1] = ' ';
    }
}

void GetRoot(char* GivenFil, int len_GivenFil, char* RootName, int len_RootName) {
    // IF ( ( TRIM( GivenFil ) == "." ) .OR. ( TRIM( GivenFil ) == ".." ) ) THEN
    //    RootName = TRIM( GivenFil )
    //    RETURN
    // END IF
    //
    // Part 1 of the proof above: the comparison blank-pads, so TRIM on its left
    // is a no-op and the whole of GivenFil is the left operand.
    //
    // `RootName = TRIM(GivenFil)` is NOT a no-op in the same way -- an
    // ASSIGNMENT copies min(LEN(src), LEN(dst)) bytes -- but it is here, because
    // the bytes TRIM would drop are blanks and char_assign blank-fills whatever
    // it does not copy. Both forms write the same buffer.
    if (char_eq(GivenFil, len_GivenFil, ".", 1) ||
        char_eq(GivenFil, len_GivenFil, "..", 2)) {
        char_assign(RootName, len_RootName, GivenFil, len_GivenFil);
        return;
    }

    // More-normal cases.  DO I=LEN_TRIM( GivenFil ),1,-1  -- part 2 of the proof.
    for (int I = len_GivenFil; I >= 1; --I) {

        if (GivenFil[I - 1] == '.') {

            if (I < len_GivenFil) {                             // Make sure the index I is okay -- part 3 of the proof
                // INDEX( '\/', GivenFil(I+1:I+1) ) -- note the operand order:
                // the two-character SET is the string being searched and the
                // single character after the dot is the substring. The set is
                // `Separators`, declared above.
                if (index_first(Separators, (int)sizeof Separators, GivenFil[(I + 1) - 1]) == 0) {  // Make sure we don't have the RootName in a different directory
                    // RootName = GivenFil(:I-1)
                    char_assign(RootName, len_RootName, GivenFil, I - 1);
                } else {
                    // RootName = GivenFil  -- This does not have a file
                    // extension. The FULL declared length, not the trimmed one.
                    char_assign(RootName, len_RootName, GivenFil, len_GivenFil);
                }
            } else {
                if (I == 1) {
                    // RootName = ''  -- a zero-length source, so char_assign
                    // blank-fills the whole of RootName.
                    //
                    // DEAD CODE IN UPSTREAM ROSCO, transcribed anyway (P7).
                    // Reaching it needs I == 1 and I == LEN(GivenFil), i.e.
                    // GivenFil is the single character '.', and the special case
                    // at the top of this function has already returned on that.
                    // No input can execute this line, so no mutant of it can be
                    // observed: mutation/GetRoot.equivalences.json declares that
                    // one and says so.
                    char_assign(RootName, len_RootName, "", 0);
                } else {
                    // RootName = GivenFil(:I-1)
                    char_assign(RootName, len_RootName, GivenFil, I - 1);
                }
            }

            return;

        }
    } // I

    // RootName = GivenFil -- again the FULL declared length. This is the only
    // statement any of the 27 gate scenarios reaches: every one of them passes
    // a name with no '.' in it, so GetRoot is the identity on the whole
    // exercised domain. See plan.json's `observability`.
    char_assign(RootName, len_RootName, GivenFil, len_GivenFil);
}
