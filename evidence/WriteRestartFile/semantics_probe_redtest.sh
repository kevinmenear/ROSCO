#!/bin/bash
# Red tests for the semantics probe. X4: a control taken at face value on its
# first use is not a control.
#
#   bash evidence/WriteRestartFile/semantics_probe_redtest.sh <label> <sed-expr>
#
# Perturbs the SHIPPED translation, re-runs the probe, reports how many BIN
# bytes and how many TXT records differ, then reverts. Wrapped in
# mutate_guarded.sh by the caller.
set -uo pipefail
LABEL="${1:?usage: semantics_probe_redtest.sh <label> <sed-expression>}"
EXPR="${2:?}"
ROOT="$(git rev-parse --show-toplevel)"
CPP="$ROOT/rosco/controller/src/writerestartfile.cpp"
BEFORE="$(git -C "$ROOT" hash-object "$CPP")"
cp "$CPP" "/tmp/wrf.cpp.$LABEL.orig"

echo "=== probe red test: $LABEL"
echo "    sed: $EXPR"
sed -i.bak "$EXPR" "$CPP" && rm -f "$CPP.bak"
if cmp -s "$CPP" "/tmp/wrf.cpp.$LABEL.orig"; then
    echo "REFUSING: the sed expression changed nothing." >&2
    cp "/tmp/wrf.cpp.$LABEL.orig" "$CPP"; exit 2
fi

# THE BIND-MOUNT RULE (RUNBOOK, units #23 and #30), and it BIT HERE. `sed -i.bak`
# unlinks and recreates the file, and the container read the gap: red tests B and
# C both died with `fatal error: writerestartfile.cpp: No such file or directory`
# while the file sat on the host, restored and hashing correctly. So prove the
# content ARRIVED from inside the container, and touch it, before compiling.
for attempt in 1 2 3 4 5; do
    if docker exec "${VIT_CONTAINER:-vit-dev}" bash -lc \
        "md5sum /workspace/ROSCO-r2/rosco/controller/src/writerestartfile.cpp && \
         touch /workspace/ROSCO-r2/rosco/controller/src/writerestartfile.cpp"; then
        break
    fi
    echo "  (the container cannot see the .cpp yet; attempt $attempt)"
    sleep 1
done

docker exec "${VIT_CONTAINER:-vit-dev}" bash -lc '
set -e
cd /workspace/ROSCO-r2/evidence/WriteRestartFile
g++ -std=c++17 -ffp-contract=off -I/workspace/ROSCO-r2/rosco/controller/src \
    -o semantics_probe_c semantics_probe.cpp
./semantics_probe_c
echo -n "  BIN bytes differing: "
cmp -l semantics_probe.f.bin semantics_probe.c.bin 2>/dev/null | wc -l
echo -n "  TXT records differing: "
diff <(cat semantics_probe.f.txt) <(cat semantics_probe.c.txt) | grep -c "^<" || true
'
RC=$?
cp "/tmp/wrf.cpp.$LABEL.orig" "$CPP"
AFTER="$(git -C "$ROOT" hash-object "$CPP")"
[ "$AFTER" = "$BEFORE" ] || { echo "RESTORE FAILED: $AFTER != $BEFORE" >&2; exit 3; }
# Leave the probe outputs as the GREEN run wrote them, under the same rule.
docker exec "${VIT_CONTAINER:-vit-dev}" bash -lc \
    "md5sum /workspace/ROSCO-r2/rosco/controller/src/writerestartfile.cpp && \
     touch /workspace/ROSCO-r2/rosco/controller/src/writerestartfile.cpp" >/dev/null
docker exec "${VIT_CONTAINER:-vit-dev}" bash -lc '
cd /workspace/ROSCO-r2/evidence/WriteRestartFile
g++ -std=c++17 -ffp-contract=off -I/workspace/ROSCO-r2/rosco/controller/src \
    -o semantics_probe_c semantics_probe.cpp && ./semantics_probe_c'
exit $RC
