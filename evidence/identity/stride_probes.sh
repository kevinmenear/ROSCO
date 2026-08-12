#!/bin/bash
# Three defects `harness/cppmutate.py` CANNOT GENERATE, run by hand against the
# same 29-case corpus and counted.
#
# `_OPERAND` is `identifier | number`, so a PARENTHESISED operand matches
# nothing: `(j - 1) * n` produces no arith_op, no drop_factor and no
# swap_operands mutant, and the stride multiplier of this translation is
# unmutated. The parenthesis is not incidental -- `exponent-grouping`, one of
# VIT's own checks, REQUIRES it.
#
# Run from the repo root, with the pre-integration harness already generated:
#   bash scripts/harness.sh identity Functions identity \
#        rosco/controller/src/Functions.f90 --against translation \
#        --out harness/identity.json
#   bash evidence/identity/stride_probes.sh
#
# NOTE `make -s test`, not `make -s`. The generated Makefile's FIRST target is
# `identity.hpp`, so a bare `make` copies the header and relinks nothing, and
# `./test` then reports the PREVIOUS translation's verdict -- unit #18's kernel
# finding, in the harness Makefile. All three probes read "failed 0" that way.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
CPP=translations/Functions/identity.cpp
TESTDIR=translations/Functions/identity_test
cp "$CPP" /tmp/identity.probe.orig.cpp
trap 'cp /tmp/identity.probe.orig.cpp "$CPP"' EXIT

probe() {
    python3 - "$1" "$2" <<'EOF'
import sys
s = open('/tmp/identity.probe.orig.cpp').read()
open('translations/Functions/identity.cpp', 'w').write(s.replace(sys.argv[1], sys.argv[2]))
EOF
    # `|| true`: `./test` EXITS NON-ZERO when cases fail, which is the whole
    # point of a probe. Under `set -e` the first probe killed the script and it
    # wrote an empty artifact -- the shape unit #20 records for the
    # pre-integration harness, met again in a five-line runner.
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$TESTDIR && make -s test >/dev/null 2>&1 && ./test" \
        > /tmp/identity.probe.json 2>&1 || true
    printf '%-22s %s\n' "$3" "$(python3 -c "
import json; d = json.load(open('/tmp/identity.probe.json'))
print('checked', d['checked'], ' failed', d['failed'],
      ' cases', [m['case'] for m in d['mismatches']])")"
}

probe '(j - 1) * n'              '(j - 1) * 3'              'stride-literal-3'
probe '(j - 1) * n'              '(j - 1) / n'              'stride-divide'
probe '(j - 1) * n + (i - 1)'    '(i - 1) * n + (j - 1)'    'row-major-transpose'
