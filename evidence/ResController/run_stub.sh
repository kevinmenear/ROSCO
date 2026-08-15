#!/bin/bash
# Run one stub through the committed kernel and keep its stdout.
#
#   bash evidence/ResController/run_stub.sh <stub.cpp> <log-name>
#
# Copied from evidence/PIController/run_stub.sh (unit #33) with two names
# changed. The rules it enforces are RUNBOOK rules, not taste:
#   * the .hpp is hash-verified from INSIDE the container after the copy --
#     a bind-mounted file has been read half-written twice in this campaign
#     (units #23 and #30), and a stub run whose input nobody checked measures
#     an unknown program;
#   * `rm -f rescontroller.o kernel.exe` before `make -s build` -- a bare
#     `make` reports the PREVIOUS translation's verdict (unit #18);
#   * stdout is redirected AT THE TIME, because kernel/ is untracked and
#     reset_to_clean.sh removes it (unit #17/#18);
#   * `touch` after the hash agrees -- unit #30's second dispatch found `make`
#     calling an object up to date against a source the bind mount had not
#     finished delivering.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="$1"; NAME="$2"
K=kernel/ResController

cp "$SRC" "$K/rescontroller.hpp"
want=$(md5 -q "$SRC")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$K/rescontroller.hpp | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$SRC" "$K/rescontroller.hpp"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH: $want != $got"; exit 1; }
docker exec vit-dev bash -lc "touch /workspace/ROSCO-r2/$K/rescontroller.hpp"

{
    echo "# stub:   $SRC"
    echo "# md5:    $want   (verified inside the container)"
    echo "#"
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$K && rm -f rescontroller.o kernel.exe && make -s build && ./kernel.exe" 2>&1
} > "evidence/ResController/$NAME"
grep -E "verification-passed|Total number of verification cases|FAILED|PASSED" "evidence/ResController/$NAME" | head -5
echo "  IDENTICAL rows:     $(grep -c 'is IDENTICAL' "evidence/ResController/$NAME" || true)"
echo "  NOT IDENTICAL rows: $(grep -c 'NOT IDENTICAL' "evidence/ResController/$NAME" || true)"
