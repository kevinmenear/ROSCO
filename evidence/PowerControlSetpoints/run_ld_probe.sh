#!/bin/bash
# The oracle for `WRITE(401,*)`: gfortran's own records against the SHIPPED
# translation's formatter, 22,526 values.
#
#   bash evidence/PowerControlSetpoints/run_ld_probe.sh   [> is NOT used; see below]
#
# WHY THIS EXISTS. Nothing else in the campaign compares this output. The kernel
# cannot -- the arm is dead in all 27 scenarios. The gate cannot -- it compares
# `avrSWAP` channels. The differential harness cannot -- it compares the mapped
# signature, and no signature carries a file. The split-file probe
# (probes/split_fort401.cpp) does compare it, and over the harness's corpus that
# is SEVENTEEN records of one value. This is the instrument that puts the
# formatter under a domain.
#
# THE TESTED CODE IS SLICED OUT OF THE TRANSLATION, NOT COPIED. `list_directed_real`
# and the two helpers it calls are cut from
# `translations/ControllerBlocks/powercontrolsetpoints.cpp` between the marker
# lines below and compiled as they stand. A probe carrying its own copy would go
# stale the first time the translation changed and would still report green --
# which is the shape of defect this campaign keeps finding, so it is not used
# here.
#
# THE REFERENCE IS REGENERATED, NOT COMMITTED. `list_directed_corpus.f90` writes
# 22,526 doubles twice -- as list-directed records to `fort.401` and as raw bits
# to `vals.bin` -- in two seconds. Committing 780 KB of binary would make the
# oracle a file nobody can regenerate a claim about; this way the corpus is the
# .f90 that is committed, and the run is reproducible from it.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
W=/workspace/ROSCO-r2
D=evidence/PowerControlSetpoints
OUT="$D/ld_probe.txt"

# 1. Slice the formatter out of the translation.
python3 - "$ROOT/translations/ControllerBlocks/powercontrolsetpoints.cpp" \
         "$ROOT/$D/ld_probe.slice.cpp" <<'PYEOF'
import re, sys
src = open(sys.argv[1]).read()
# From `field(` to the end of `list_directed_real`. Anchored on the two function
# signatures rather than on line numbers, which move with every comment edit.
a = src.index("std::string field(const std::string& text, int w) {")
b = src.index("// The unit itself.")
slice_ = src[a:b].rstrip()
for want in ("nonfinite_text", "list_directed_real", '"%.16E"', "%.*f"):
    assert want in slice_, f"the slice does not contain {want}: markers moved"
open(sys.argv[2], "w").write(slice_ + "\n")
print(f"sliced {len(slice_.splitlines())} lines from the translation")
PYEOF

# 2. A main() around the slice. The slice is #included rather than pasted, so
#    what runs is the file step 1 produced and nothing edited afterwards.
cat > "$ROOT/$D/ld_probe.cpp" <<'EOF'
// Generated wrapper. Do not edit -- run_ld_probe.sh rewrites it.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
namespace {
#include "ld_probe.slice.cpp"
}  // namespace
int main() {
    std::FILE* fb = std::fopen("vals.bin", "rb");
    std::FILE* fr = std::fopen("fort.401", "rb");
    if (!fb || !fr) { std::fprintf(stderr, "probe: corpus missing\n"); return 2; }
    long n = 0, bad = 0;
    double v;
    char ref[64];
    while (std::fread(&v, sizeof v, 1, fb) == 1) {
        if (std::fgets(ref, sizeof ref, fr) == nullptr) {
            std::fprintf(stderr, "probe: fewer records than values at %ld\n", n);
            return 2;
        }
        size_t len = std::strlen(ref);
        while (len && (ref[len - 1] == '\n' || ref[len - 1] == '\r')) ref[--len] = '\0';
        const std::string got = list_directed_real(v);
        if (got != std::string(ref, len)) {
            if (bad < 10) {
                std::printf("MISMATCH %ld  value %.17g\n  gfortran [%s]\n  C++      [%s]\n",
                            n, v, ref, got.c_str());
            }
            ++bad;
        }
        ++n;
    }
    if (std::fgets(ref, sizeof ref, fr) != nullptr) {
        std::fprintf(stderr, "probe: more records than values\n");
        return 2;
    }
    std::printf("values compared %ld\nmismatched     %ld\n", n, bad);
    return bad == 0 ? 0 : 1;
}
EOF

# 3. Regenerate the reference corpus, build, run. Stdout is captured by the
#    heredoc's redirect INSIDE this script and not by the caller's `>`, for the
#    reason capture_done_check.sh states: a redirect made by the caller
#    truncates its target before the run and dirties the tree.
{
    echo "# gfortran's list-directed REAL(8) records against the SHIPPED formatter"
    echo "# sliced from translations/ControllerBlocks/powercontrolsetpoints.cpp"
    echo "# corpus regenerated from evidence/PowerControlSetpoints/list_directed_corpus.f90"
    echo "#"
    docker exec "$CONTAINER" bash -lc "
        set -e
        mkdir -p /tmp/ld_probe && cd /tmp/ld_probe
        cp $W/$D/list_directed_corpus.f90 $W/$D/ld_probe.cpp $W/$D/ld_probe.slice.cpp .
        gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off -O2 \
            list_directed_corpus.f90 -o corpus
        rm -f fort.401 vals.bin
        ./corpus
        g++ -O2 -ffp-contract=off -I. ld_probe.cpp -o probe
        ./probe" 2>&1
} > "$OUT"
rc=$?
cat "$OUT"
exit $rc
