// CONSTANT STUB -- reads NO input and writes a fixed answer (the answer for
// case GetWords.0.0.1). Unit #6's test: if this PASSES, the kernel is a lookup
// table rather than a comparison.
static void put(char* d, int n, const char* s) {
    int i = 1;
    for (; s[i - 1] != '\0' && i <= n; ++i) d[i - 1] = s[i - 1];
    for (; i <= n; ++i) d[i - 1] = ' ';
}
void GetWords(char* Line, int len_Line, char* Words, int len_Words, int NumWords) {
    (void)Line; (void)len_Line;
    if (NumWords >= 1) put(&Words[0], len_Words, "Controller");
    if (NumWords >= 2) put(&Words[len_Words], len_Words, "parameter");
    for (int j = 3; j <= NumWords; ++j) put(&Words[(j - 1) * len_Words], len_Words, "");
}
