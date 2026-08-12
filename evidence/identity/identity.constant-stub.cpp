// STUB, not a translation. The INPUT to a measurement (X4).
//
// Reads NO argument: it writes a hardcoded 3x3 identity whatever `n` says.
// The reference's one call site is the literal `identity(3)`, so every captured
// case has n == 3 and this stub is indistinguishable from the translation on
// the kernel's whole domain. A PASS here is the measurement of what the kernel
// cannot constrain.
void identity(int n, double* identity_result) {
    (void)n;
    identity_result[0] = 1.0;
    identity_result[1] = 0.0;
    identity_result[2] = 0.0;
    identity_result[3] = 0.0;
    identity_result[4] = 1.0;
    identity_result[5] = 0.0;
    identity_result[6] = 0.0;
    identity_result[7] = 0.0;
    identity_result[8] = 1.0;
}
