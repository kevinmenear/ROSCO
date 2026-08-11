// PADDING STUB. The digits are RIGHT; every blank-fill byte is 'X'.
// The caller writes TRIM(Int2LStr(I)) into OL_String, and TRIM removes trailing
// blanks -- so this asks whether the kernel can see the 10 of 11 bytes the unit
// writes that are not the digit.
void Int2LStr(int Num, char* Int2LStr_result) {
    for (int i = 0; i < 11; ++i) Int2LStr_result[i] = 'X';
    long long mag = (Num < 0) ? -(long long)Num : (long long)Num;
    int first = 11;
    do { Int2LStr_result[--first] = (char)('0' + (int)(mag % 10)); mag /= 10; } while (mag != 0);
    if (Num < 0) Int2LStr_result[--first] = '-';
    int lead = first;
    for (int i = 0; lead + i < 11; ++i) Int2LStr_result[i] = Int2LStr_result[lead + i];
    for (int i = 11 - lead; i < 11; ++i) Int2LStr_result[i] = 'X';
}
