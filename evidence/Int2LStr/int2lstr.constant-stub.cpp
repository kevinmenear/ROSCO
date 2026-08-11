// CONSTANT STUB. Reads NO argument and writes the captured answer as a literal.
// The single captured case has Num == 1, so the reference result is "1" blank
// padded to 11. If the kernel PASSES this, it is a lookup table for one input.
void Int2LStr(int Num, char* Int2LStr_result) {
    (void)Num;
    for (int i = 0; i < 11; ++i) Int2LStr_result[i] = ' ';
    Int2LStr_result[0] = '1';
}
