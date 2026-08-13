#!/bin/bash
# Run one stub through the committed kernel and keep its stdout.
#
#   bash evidence/wrap_360/run_stub.sh <stub.cpp> <log-name>
#
# Copied from evidence/wrap_180/run_stub.sh with the unit name changed; the
# three rules it encodes are RUNBOOK rules and not taste:
#   * the .hpp is hash-verified from INSIDE the container after the copy --
#     a bind-mounted file has been read half-written twice in this campaign
#     (unit #23), and a stub run whose input nobody checked measures an
#     unknown program;
#   * `rm -f wrap_360.o kernel.exe` before `make -s build` -- a bare `make`
#     reports the PREVIOUS translation's verdict (unit #18);
#   * stdout is redirected AT THE TIME, because kernel/ is untracked and
#     reset_to_clean.sh removes it (unit #17/#18).
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="$1"; NAME="$2"
K=kernel/wrap_360

cp "$SRC" "$K/wrap_360.hpp"
want=$(md5 -q "$SRC")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$K/wrap_360.hpp | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$SRC" "$K/wrap_360.hpp"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH: $want != $got"; exit 1; }

{
    echo "# stub:   $SRC"
    echo "# md5:    $want   (verified inside the container)"
    echo "#"
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$K && rm -f wrap_360.o kernel.exe && make -s build && ./kernel.exe" 2>&1
} > "evidence/wrap_360/$NAME"
grep -E "verification-passed|Total number of verification cases|FAILED|PASSED" "evidence/wrap_360/$NAME" | head -5
echo "  IDENTICAL rows:     $(grep -c 'is IDENTICAL' "evidence/wrap_360/$NAME" || true)"
echo "  NOT IDENTICAL rows: $(grep -c 'NOT IDENTICAL' "evidence/wrap_360/$NAME" || true)"
