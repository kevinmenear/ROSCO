// VIT Translation Scaffold
// Function: GetPath
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE GetPath(GivenFil, PathName)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 9940672f0b3c
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T14:08:39Z

#include <algorithm>

// GivenFil and PathName arrive as CHARACTER(KIND=C_CHAR) arrays of exactly
// len_GivenFil and len_PathName bytes. Neither is NUL-terminated, so those two
// parameters are the only statement of their extents, and each is named once.
//
// PathSep is `CHARACTER(1), PARAMETER :: PathSep = '/'`, from
// rosco/controller/src/SysFiles/SysGnuLinux.f90 -- the file
// rosco/controller/CMakeLists.txt:93 selects for a GNU/Linux build. It is a
// COMPILE-TIME parameter in the reference, so it is a compile-time constant
// here. On a Windows build the reference compiles SysGnuWin.f90 and PathSep is
// '\\'; that is a property of the BUILD, not of this translation, and it is
// written down so the next platform does not have to rediscover it.
static const char PathSep = '/';

// INDEX(string, substring, BACK=.TRUE.), for a one-character substring: the
// 1-based position of the LAST occurrence, or 0 when there is none. Searched
// over the FULL declared length -- Fortran's INDEX does not trim, so trailing
// blanks are part of the search space and `len` is the bound, not a trimmed
// length.
//
// One function, called twice, because the Fortran calls one intrinsic twice.
// Two transcribed copies would be two sets of mutable sites computing the same
// quantity, which is the shape units #1 and #4 each paid a mutation score for.
static int index_back(const char* s, int len, char c) {
    for (int i = len; i >= 1; --i) {
        if (s[i - 1] == c) {
            return i;
        }
    }
    return 0;
}

// A Fortran CHARACTER assignment `dst = src` is not a copy. It transfers
// min(LEN(src), LEN(dst)) bytes and BLANK-FILLS the remainder of dst; when src
// is the longer it truncates silently rather than erroring. Both branches of
// GetPath are such an assignment, so the rule is written once here. Writing the
// fill bound at each of the two call sites would restate a quantity that
// `len_dst` already fixes -- unit #1's "name a size once".
// BOTH LOOPS ARE 1-BASED, and that is unit #1's lesson rather than a style
// choice. Written 0-based -- `for (i = 0; i < n; ++i)` and
// `for (i = n; i < len_dst; ++i)` -- BOTH boundary mutants SURVIVED (score
// 0.882, mutation/GetPath.survivors_0based.json): `<` -> `<=` on the first loop
// writes dst[n], which the second loop immediately overwrites, and on the
// second it writes dst[len_dst], one byte past the buffer. Neither is a wrong
// ANSWER, so no value comparison can see either.
//
// 1-based, the boundary mutant LEAVES A BYTE UNWRITTEN instead of writing one
// past the end -- dst keeps its incoming byte, the comparison sees it, and both
// mutants die. Same fact, opposite observability.
static void char_assign(char* dst, int len_dst, const char* src, int len_src) {
    const int n = std::min(len_src, len_dst);
    for (int i = 1; i <= n; ++i) {
        dst[i - 1] = src[i - 1];
    }
    for (int i = n + 1; i <= len_dst; ++i) {
        dst[i - 1] = ' ';
    }
}

void GetPath(char* GivenFil, int len_GivenFil, char* PathName, int len_PathName) {
    // I = INDEX( GivenFil, '\', BACK=.TRUE. )
    // I = MAX( I, INDEX( GivenFil, '/', BACK=.TRUE. ) )
    //
    // The forward-slash search takes the LITERAL '/', not PathSep. On this
    // build they are the same byte and the distinction is invisible; in the
    // reference they are two different things -- GetPath hard-codes both
    // separators when it SEARCHES and uses the platform parameter only when it
    // CONSTRUCTS the fallback below. Collapsing them would change behaviour on
    // a Windows build, where PathSep is '\\'. P7: transcribe what is written.
    int I = index_back(GivenFil, len_GivenFil, '\\');
    I = std::max(I, index_back(GivenFil, len_GivenFil, '/'));

    if (I == 0) {
        // we don't have a path specified, return '.'
        // PathName = '.'//PathSep -- a two-character source. Its length is the
        // array's own size rather than a repeated literal.
        const char dot_sep[] = { '.', PathSep };
        char_assign(PathName, len_PathName, dot_sep, (int)sizeof dot_sep);
    } else {
        // PathName = GivenFil(:I)
        char_assign(PathName, len_PathName, GivenFil, I);
    }
}
