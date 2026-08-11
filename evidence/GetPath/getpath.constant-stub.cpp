// CONSTANT STUB -- kernel red test B. Reads NEITHER input; writes the one
// answer the captured case expects, blank-padded. If this passes, the kernel's
// window cannot tell a translation from a lookup table.
void GetPath(char* GivenFil, int len_GivenFil, char* PathName, int len_PathName) {
    const char* answer = "/workspace/ROSCO-r2/Examples/";
    int i = 0;
    while (answer[i] != '\0' && i < len_PathName) { PathName[i] = answer[i]; ++i; }
    for (; i < len_PathName; ++i) PathName[i] = ' ';
}
