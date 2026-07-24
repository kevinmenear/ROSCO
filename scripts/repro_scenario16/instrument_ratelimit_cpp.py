#!/usr/bin/env python3
"""Insert an operand-capture dump into the integrated C++ ratelimit
(src/Functions/ratelimit.cpp). Mirrors the Fortran instrumenter: dumps exact
IEEE bit patterns for every call whose result is near the zero crossing,
captured before LastSignal is overwritten.

Usage: instrument_ratelimit_cpp.py <path-to-ratelimit.cpp> <dump-file-path>
"""
import sys
src, dump = sys.argv[1], sys.argv[2]
s = open(src).read()
if "VIT_REPRO_DUMP" in s:
    print("already instrumented"); sys.exit(0)
anchor = "        result = rlP->LastSignal[idx] + rate * DT;\n"
assert anchor in s, "anchor not found (source shape changed?)"
blk = anchor + f'''        // --- VIT_REPRO_DUMP (C++) ---
        if (result < 1e-9 && result > -1e-9) {{
            FILE* fp = fopen("{dump}", "a");
            auto B = [](double d){{ unsigned long long u; __builtin_memcpy(&u,&d,8); return u; }};
            fprintf(fp, "%d %016llX %016llX %016llX %016llX %016llX %016llX %016llX\\n",
                *inst, B(inputSignal), B(rlP->LastSignal[idx]), B(DT), B(rate),
                B(result), B(minRate), B(maxRate));
            fclose(fp);
        }}
        // --- end VIT_REPRO_DUMP ---
'''
s = s.replace(anchor, blk, 1)
for inc in ("#include <cstdio>", "#include <cstring>"):
    if inc not in s: s = inc + "\n" + s
open(src, "w").write(s)
print("instrumented", src)
