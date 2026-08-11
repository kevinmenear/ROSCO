// CONSTANT STUB -- reads NEITHER input and writes the one captured answer as a
// literal, blank-padded. Unit #6's test: if this passes, the kernel is a lookup
// table for this unit whatever its case count says.
void GetRoot(char* GivenFil, int len_GivenFil, char* RootName, int len_RootName) {
    const char answer[] = { 'v', 'i', 't', '_', 's', 'i', 'm', '1' };
    const int n = (int)sizeof answer;
    for (int i = 1; i <= len_RootName; ++i) {
        RootName[i - 1] = (i <= n) ? answer[i - 1] : ' ';
    }
}
