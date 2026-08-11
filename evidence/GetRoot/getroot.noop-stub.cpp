// NO-OP STUB -- reads no argument and writes nothing.
//
// Unit #6's no-op stub scored OUT_TOL and so demonstrated that the kernel
// instrument could move. This unit's call site is `CALL GetRoot(RootName,
// RootName)` -- the SAME variable is both dummies -- so the captured input value
// of `rootname` IS the captured reference output. Whether that makes a no-op
// pass is the measurement; the artifact beside this file records the answer.
void GetRoot(char* GivenFil, int len_GivenFil, char* RootName, int len_RootName) {
}
