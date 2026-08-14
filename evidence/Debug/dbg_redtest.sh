#!/bin/bash
# Red tests for scripts/dbgcheck.py, on unit Debug. X4.
#
#   bash evidence/Debug/dbg_redtest.sh <label> <sed-expression>
#
# Perturbs the SHIPPED translation (rosco/controller/src/debug.cpp), rebuilds,
# installs, runs the same 27 scenarios the green ran, and compares the archive
# against `pre` -- the reference taken with Debug still Fortran. Then reverts
# and rebuilds, so the tree is left as it was found.
#
# THE BIND-MOUNT RULE, which this campaign has paid for twice (RUNBOOK, units
# #23 and #30): a host-side edit is not necessarily visible to `make` inside the
# container when the rebuild is seconds later. `make` stats the file, sees the
# PRE-EDIT content, calls the object up to date, and the red test reports a
# perturbation that was never compiled -- which reads as "the instrument is
# blind" rather than as "the edit did not arrive". So the edit is hashed FROM
# INSIDE the container and then touched there, before anything is built.
#
# Wrapped in mutate_guarded.sh by the caller, so a kill leaves a marker rather
# than a perturbed translation.
set -euo pipefail

LABEL="${1:?usage: dbg_redtest.sh <label> <sed-expression>}"
EXPR="${2:?}"

ROOT="$(git rev-parse --show-toplevel)"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"
CPP="$ROOT/rosco/controller/src/debug.cpp"

BEFORE="$(git -C "$ROOT" hash-object "$CPP")"
cp "$CPP" "/tmp/debug.cpp.$LABEL.orig"

build() {
    docker exec "$CONTAINER" bash -lc \
        "cd $WORKDIR/rosco/controller/build && cmake --build . -j4 >/tmp/rt_build.log 2>&1 && \
         cp libdiscon.so $WORKDIR/rosco/lib/libdiscon.so" \
        || { echo "BUILD FAILED"; docker exec "$CONTAINER" tail -20 /tmp/rt_build.log; exit 1; }
}

echo "=== red test: $LABEL"
echo "    sed: $EXPR"
sed -i.bak "$EXPR" "$CPP" && rm -f "$CPP.bak"
if cmp -s "$CPP" "/tmp/debug.cpp.$LABEL.orig"; then
    echo "REFUSING: the sed expression changed nothing. A perturbation that is"
    echo "not applied reports the instrument blind when it is the edit that is"
    echo "missing." >&2
    exit 2
fi

# The two halves of the bind-mount rule: prove the content arrived, then make
# sure the object cannot be older than the source it is built from.
docker exec "$CONTAINER" bash -lc "md5sum $WORKDIR/rosco/controller/src/debug.cpp && touch $WORKDIR/rosco/controller/src/debug.cpp"
build

python3 "$ROOT/scripts/dbgcheck.py" capture --label "$LABEL" >/dev/null
set +e
python3 "$ROOT/scripts/dbgcheck.py" compare --a pre --b "$LABEL" \
        --out "$ROOT/evidence/Debug/dbg.redtest.$LABEL.json" | tail -3
RC=$?
set -e

cp "/tmp/debug.cpp.$LABEL.orig" "$CPP"
AFTER="$(git -C "$ROOT" hash-object "$CPP")"
[ "$AFTER" = "$BEFORE" ] || { echo "RESTORE FAILED: $AFTER != $BEFORE" >&2; exit 3; }
docker exec "$CONTAINER" bash -lc "md5sum $WORKDIR/rosco/controller/src/debug.cpp && touch $WORKDIR/rosco/controller/src/debug.cpp"
build

if [ "$RC" -eq 0 ]; then
    echo "RED TEST '$LABEL' STAYED GREEN -- the instrument did not see it."
    exit 1
fi
echo "red test '$LABEL': went red, as required (exit $RC)"
