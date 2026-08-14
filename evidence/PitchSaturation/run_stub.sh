#!/bin/bash
# Run one stub through the committed kernel and keep its stdout.
#
#   bash evidence/PitchSaturation/run_stub.sh <stub.cpp> <log-name>
#
# Copied from evidence/PIIController/run_stub.sh (unit #35) with the unit name
# changed; that file came from evidence/PIController/ (#33) and evidence/saturate/ (#24)
# and one line added; the three rules it enforces are RUNBOOK rules, not taste:
#   * the .hpp is hash-verified from INSIDE the container after the copy --
#     a bind-mounted file has been read half-written twice in this campaign
#     (units #23 and #30), and a stub run whose input nobody checked measures
#     an unknown program;
#   * `rm -f pitchsaturation.o kernel.exe` before `make -s build` -- a bare `make`
#     reports the PREVIOUS translation's verdict (unit #18);
#   * stdout is redirected AT THE TIME, because kernel/ is untracked and
#     reset_to_clean.sh removes it (unit #17/#18).
#
# The added line is `touch`: unit #30's second dispatch found `make` calling an
# object up to date against a source the bind mount had not finished delivering,
# and the object cannot be older than its source if the source is touched after
# the hash agrees.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="$1"; NAME="$2"
K=kernel/PitchSaturation

cp "$SRC" "$K/pitchsaturation.hpp"
want=$(md5 -q "$SRC")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$K/pitchsaturation.hpp | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$SRC" "$K/pitchsaturation.hpp"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH: $want != $got"; exit 1; }
docker exec vit-dev bash -lc "touch /workspace/ROSCO-r2/$K/pitchsaturation.hpp"

{
    echo "# stub:   $SRC"
    echo "# md5:    $want   (verified inside the container)"
    echo "#"
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$K && rm -f pitchsaturation.o kernel.exe && make -s build && ./kernel.exe" 2>&1
} > "evidence/PitchSaturation/$NAME"
grep -E "verification-passed|Total number of verification cases|FAILED|PASSED" "evidence/PitchSaturation/$NAME" | head -5
echo "  IDENTICAL rows:     $(grep -c 'is IDENTICAL' "evidence/PitchSaturation/$NAME" || true)"
echo "  NOT IDENTICAL rows: $(grep -c 'NOT IDENTICAL' "evidence/PitchSaturation/$NAME" || true)"
