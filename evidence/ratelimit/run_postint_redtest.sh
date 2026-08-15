#!/bin/bash
# ONE post-integration red test per invocation, for unit #46 `ratelimit`.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/ratelimit/run_postint_redtest.sh '<from>' '<to>' <out.json> "<what was perturbed>"
#
# The post-integration layer is the ONLY one that reads the WRAPPER. So its red
# test must perturb the wrapper, not the translation -- and it must REBUILD
# BETWEEN THE EDIT AND THE RUN, which is the step units #23 and #34 both record
# having been skipped once: `harness.sh --post-integration` links the installed
# library, and without a rebuild it links the library the previous run built and
# reports the previous run's verdict.
#
# The EXIT trap reverts the wrapper AND rebuilds, unconditionally. A
# perturbation left in the tree is a corrupted gate, and a REVERTED source with
# a perturbed .so still installed is worse -- it looks clean and is not. The
# green is re-taken after the revert by the caller, not asserted here.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/Functions.f90
FROM="$1"; TO="$2"; OUT="$3"; WHAT="$4"

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j\$(nproc) > /dev/null 2>&1 \
         && cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so && echo built"
}
trap 'git checkout -- "$F"; echo "reverted $F"; build' EXIT

n=$(grep -c -- "$FROM" "$F" || true)
[ "$n" = "1" ] || { echo "perturb-from matched $n time(s) in $F, expected 1" >&2; exit 2; }
python3 - "$F" "$FROM" "$TO" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); t = p.read_text()
assert t.count(sys.argv[2]) == 1
p.write_text(t.replace(sys.argv[2], sys.argv[3]))
PY
build

bash scripts/harness.sh ratelimit Functions ratelimit "$F" \
    --post-integration --out "$OUT" --red-test "$WHAT" 2>&1 | tail -12
