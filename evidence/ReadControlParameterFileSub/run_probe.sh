#!/bin/bash
# Build and run the standalone differential probe for ReadControlParameterFileSub.
#
#   bash evidence/ReadControlParameterFileSub/run_probe.sh
#   bash evidence/ReadControlParameterFileSub/run_probe.sh --red-test <sed-expr> <label>
#
# WHY THIS IS A SCRIPT. The probe is the unit's ONLY instrument -- the
# differential harness refused and integration has no copy-back for the ~50
# ALLOCATABLE outputs, so the gate cannot run either. An instrument that lives
# in somebody's shell history is not an instrument.
#
# --red-test applies one `sed -E` expression to a COPY of the translation,
# rebuilds from that copy, runs, and puts the original back. The copy is what
# the build reads, so there is no window in which a perturbed file could be
# committed -- the campaign's own lesson that a runner must restore the file the
# BUILD read, not only the file it perturbed, is answered by never perturbing
# the tracked file at all.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORK="/workspace/$(basename "$ROOT")"
CPP="translations/ReadSetParameters/readcontrolparameterfilesub.cpp"

SEDX=""
LABEL="baseline"
if [ "${1-}" = "--red-test" ]; then
    SEDX="${2:?--red-test needs a sed -E expression}"
    LABEL="${3:?--red-test needs a label}"
fi

# The driver is GENERATED from ROSCO_Types.f90's own TYPE definition, every run,
# so a component added there is compared without an edit here.
python3 "$ROOT/evidence/ReadControlParameterFileSub/gen_probe.py" \
    > "$ROOT/evidence/ReadControlParameterFileSub/vit_rcpfs_probe.f90"

docker exec "$CONTAINER" bash -lc "
set -e
cd $WORK
mkdir -p /tmp/probe
cp $CPP /tmp/probe/unit.cpp
if [ -n '$SEDX' ]; then
    sed -E -i '$SEDX' /tmp/probe/unit.cpp
    if cmp -s $CPP /tmp/probe/unit.cpp; then
        echo 'run_probe: the --red-test expression changed NOTHING; refusing' >&2
        exit 3
    fi
    diff $CPP /tmp/probe/unit.cpp | head -8 || true
fi
g++ -std=c++17 -O2 -fPIC -ffp-contract=off -I rosco/controller/src \\
    -include rosco/controller/src/vit_translated.h \\
    -c /tmp/probe/unit.cpp -o /tmp/probe/rcpfs.o
python3 scripts/_integration_shim.py ReadControlParameterFileSub \\
    -f rosco/controller/src/ReadSetParameters.f90 > /tmp/probe/shim.cpp
g++ -std=c++17 -O2 -fPIC -ffp-contract=off -I rosco/controller/src \\
    -c /tmp/probe/shim.cpp -o /tmp/probe/shim.o
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off \\
    -I rosco/controller/build/ftnmods \\
    -o /tmp/probe/vit_rcpfs_probe evidence/ReadControlParameterFileSub/vit_rcpfs_probe.f90 \\
    /tmp/probe/rcpfs.o /tmp/probe/shim.o -L rosco/controller/build -ldiscon -lstdc++
cd Examples
LD_LIBRARY_PATH=$WORK/rosco/controller/build /tmp/probe/vit_rcpfs_probe DISCON*.IN \\
    2>&1 | grep -v 'ROSCO Warning' | grep -v '^        COPYBK' || true
"
echo "run_probe: $LABEL"
