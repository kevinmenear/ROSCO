// WRONG CONSTANT. Writes a result that is not the captured answer.
// If the kernel PASSES this, the comparison is not alive at all.
void Int2LStr(int Num, char* Int2LStr_result) {
    (void)Num;
    for (int i = 0; i < 11; ++i) Int2LStr_result[i] = ' ';
    Int2LStr_result[0] = '9';
}
