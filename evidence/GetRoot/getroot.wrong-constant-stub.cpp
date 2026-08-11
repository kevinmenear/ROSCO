// WRONG-CONSTANT STUB -- the control for the two stubs beside it.
//
// The no-op stub and the correct-constant stub both score 62/62 IDENTICAL. On
// their own that is ambiguous: an instrument that CANNOT MOVE would report the
// same thing. This stub writes a wrong answer, so it must score OUT_TOL. If it
// does, the kernel's comparison is alive and the other two artifacts are
// statements about this UNIT rather than about a broken check.
void GetRoot(char* GivenFil, int len_GivenFil, char* RootName, int len_RootName) {
    for (int i = 1; i <= len_RootName; ++i) {
        RootName[i - 1] = 'X';
    }
}
