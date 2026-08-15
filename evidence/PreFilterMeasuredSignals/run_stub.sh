#!/bin/bash
# Run one stub through the committed kernel and keep its stdout.
#
#   bash evidence/PreFilterMeasuredSignals/run_stub.sh <stub.cpp> <log-name>
#
# Copied from evidence/PowerControlSetpoints/run_stub.sh (unit #37) with the
# unit name changed; that file came from evidence/PIIController/ (#35), which
# came from evidence/PIController/ (#33) and evidence/saturate/ (#24). The
# three rules it enforces are RUNBOOK rules, not taste:
#   * the .hpp is hash-verified from INSIDE the container after the copy --
#     a bind-mounted file has been read half-written twice in this campaign
#     (units #23 and #30), and a stub run whose input nobody checked measures
#     an unknown program;
#   * `rm -f prefiltermeasuredsignals.o kernel.exe` before `make -s build` -- a
#     bare `make` reports the PREVIOUS translation's verdict (unit #18);
#   * stdout is redirected AT THE TIME, because kernel/ is untracked and
#     reset_to_clean.sh removes it (unit #17/#18).
#
# THE KERNEL'S CALLEES ARE THE ORIGINAL FORTRAN, not the six integrated C++
# translations: kernel/PreFilterMeasuredSignals/vit_bridge_lpfilter.f90 and its
# five siblings BIND(C, NAME='lpfilter_c') straight onto `USE Filters, ONLY:
# LPFilter`. So every number a stub moves here is this unit's own doing -- call
# order, arguments, instance counters -- and never a callee's.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SRC="$1"; NAME="$2"
K=kernel/PreFilterMeasuredSignals

cp "$SRC" "$K/prefiltermeasuredsignals.hpp"
want=$(md5 -q "$SRC")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$K/prefiltermeasuredsignals.hpp | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$SRC" "$K/prefiltermeasuredsignals.hpp"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH: $want != $got"; exit 1; }
docker exec vit-dev bash -lc "touch /workspace/ROSCO-r2/$K/prefiltermeasuredsignals.hpp"

{
    echo "# stub:   $SRC"
    echo "# md5:    $want   (verified inside the container)"
    echo "#"
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$K && rm -f prefiltermeasuredsignals.o kernel.exe && make -s build && ./kernel.exe" 2>&1
} > "evidence/PreFilterMeasuredSignals/$NAME"
grep -E "verification-passed|Total number of verification cases|FAILED|PASSED" "evidence/PreFilterMeasuredSignals/$NAME" | head -5
echo "  IDENTICAL rows:     $(grep -c 'is IDENTICAL' "evidence/PreFilterMeasuredSignals/$NAME" || true)"
echo "  NOT IDENTICAL rows: $(grep -c 'NOT IDENTICAL' "evidence/PreFilterMeasuredSignals/$NAME" || true)"
