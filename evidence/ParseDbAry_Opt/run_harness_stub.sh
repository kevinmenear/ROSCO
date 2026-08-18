#!/bin/bash
# Run ONE stub through the differential harness and keep its artifact.
#
#   bash evidence/ParseDbAry_Opt/run_harness_stub.sh <stub.cpp> <out.json> "<what was cut>"
#
# Copied from `evidence/ChkParseData/run_harness_stub.sh` (P4), same module,
# with the unit's three names changed. The shipped translation is restored on
# the way out, and the stub is HASH-VERIFIED from inside the container before
# anything is built (unit #23: `cp` onto a bind-mounted file is read
# half-written often enough to need a guard, not a warning).
#
# `--no-generate` KEEPS THE CORPUS. Unit #26's rule: a red result and the green
# it certifies must name the same case count, so every stub is scored on the
# case file the green was taken on rather than on a freshly generated one.
#
# THE TREE MUST BE CLEAN (`scripts/reset_to_clean.sh`). The pre-integration
# harness links this campaign's Fortran objects, and after integration
# `ROSCO_Helpers.f90.o` IS the wrapper -- there is no independent reference
# left to compare against.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
STUB="$1"; OUT="$2"; WHAT="$3"
LIVE=translations/ROSCO_Helpers/parsedbary_opt.cpp
KEEP=$(mktemp)
cp "$LIVE" "$KEEP"
restore() { cp "$KEEP" "$LIVE"; rm -f "$KEEP"; }
trap restore EXIT

cp "$STUB" "$LIVE"
want=$(md5 -q "$STUB")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$LIVE | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$STUB" "$LIVE"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH on $LIVE"; exit 1; }
echo "stub md5 (verified inside the container): $got  <- $STUB"

bash scripts/harness.sh ParseDbAry_Opt ROSCO_Helpers parsedbary_opt \
     rosco/controller/src/ROSCO_Helpers.f90 \
     --no-generate --out "$OUT" --red-test "$WHAT" 2>&1 | tail -6

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/translations/ROSCO_Helpers/parsedbary_opt_test && \
     make test >/dev/null 2>&1 && ./test parsedbary_opt_cases.bin" \
    > /tmp/stub_stdout.txt 2>&1 || true
python3 - "$ROOT/$OUT" "$WHAT" <<'PY'
import json, sys, pathlib
out, what = sys.argv[1], sys.argv[2]
txt = pathlib.Path('/tmp/stub_stdout.txt').read_text()
lines = txt.splitlines()
obj = None
for i in range(len(lines) - 1, -1, -1):
    if lines[i].lstrip().startswith('{'):
        try:
            obj = json.loads('\n'.join(lines[i:]))
            break
        except json.JSONDecodeError:
            continue
if obj is None:
    print('run_harness_stub: no JSON from ./test -- artifact NOT written')
    sys.exit(1)
obj['red_test'] = what
obj['red_test_note'] = ('run with --no-generate, so this is the same case file '
                        'the green in harness/ParseDbAry_Opt.json was taken on')
pathlib.Path(out).write_text(json.dumps(obj, indent=2) + '\n')
print(f"{out}: checked {obj.get('checked')}  failed {obj.get('failed')}  "
      f"inadmissible {obj.get('inadmissible')}")
PY
