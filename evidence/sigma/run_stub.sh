#!/bin/bash
# Run one stub through the committed kernel and keep its stdout.
#
#   bash evidence/sigma/run_stub.sh <stub.cpp> <log-name>
#
# Three things here are RUNBOOK rules rather than taste:
#   * the .hpp is hash-verified from INSIDE the container after the copy --
#     a bind-mounted file has been read half-written twice in this campaign
#     (unit #23), and a stub run whose input nobody checked measures an
#     unknown program;
#   * `rm -f sigma.o kernel.exe` before `make -s build` -- a bare `make`
#     reports the PREVIOUS translation's verdict (unit #18);
#   * stdout is redirected AT THE TIME, because kernel/ is untracked and
#     reset_to_clean.sh removes it (unit #17/#18).
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="$1"; NAME="$2"
K=kernel/sigma

cp "$SRC" "$K/sigma.hpp"
want=$(md5 -q "$SRC")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$K/sigma.hpp | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$SRC" "$K/sigma.hpp"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH: $want != $got"; exit 1; }

{
    echo "# stub:   $SRC"
    echo "# md5:    $want   (verified inside the container)"
    echo "#"
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$K && rm -f sigma.o kernel.exe && make -s build && ./kernel.exe" 2>&1
} > "evidence/sigma/$NAME"
grep -E "verification-passed|Total number of verification cases|FAILED|PASSED" "evidence/sigma/$NAME" | head -5
echo "  IDENTICAL rows:     $(grep -c 'is IDENTICAL' "evidence/sigma/$NAME" || true)"
echo "  NOT IDENTICAL rows: $(grep -c 'NOT IDENTICAL' "evidence/sigma/$NAME" || true)"
