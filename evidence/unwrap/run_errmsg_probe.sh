#!/bin/bash
# Run `errmsg_extremes_probe.cpp` over this unit's own 403 harness cases.
#
# The generated harness Makefile's FIRST rule is `unwrap.hpp: <the translation>`,
# so the probe is swapped over `translations/Functions/unwrap.cpp` and restored
# on the way out. `make -s test`, not a bare `make`: a bare `make` stops after
# copying the header and `./test` reports the PREVIOUS build's verdict (unit
# #22). `|| true` on the run, because `./test` exits non-zero when cases fail
# and `set -e` would otherwise leave an EMPTY artifact (unit #22).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
LIVE=translations/Functions/unwrap.cpp
KEEP=$(mktemp); cp "$LIVE" "$KEEP"
trap 'cp "$KEEP" "$LIVE"; rm -f "$KEEP"' EXIT
cp evidence/unwrap/errmsg_extremes_probe.cpp "$LIVE"
want=$(md5 -q evidence/unwrap/errmsg_extremes_probe.cpp)
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$LIVE | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp evidence/unwrap/errmsg_extremes_probe.cpp "$LIVE"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH"; exit 1; }
echo "probe md5 (verified inside the container): $got"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/translations/Functions/unwrap_test && \
    rm -f unwrap.hpp unwrap_test.o test && make -s test && \
    ./test unwrap_cases.bin 2>/tmp/probe.err >/tmp/probe.out; \
    echo \"--- the probe's counters (stderr) ---\"; cat /tmp/probe.err; \
    echo \"--- the run's own verdict (stdout) ---\"; \
    python3 -c \"import json;d=json.load(open('/tmp/probe.out'));print('checked',d['checked'],'failed',d['failed'])\"" || true
