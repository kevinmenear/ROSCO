#!/bin/bash
# Generate unit #44 `YawRateControl`'s differential-harness corpus ON THE HOST.
#
#   bash scripts/run_if_time_remains.sh 1800 \
#       bash evidence/YawRateControl/run_generate_corpus.sh [extra vit_harness args...]
#
# WHY NOT IN THE CONTAINER, which is where every other step of this campaign
# runs. `vit-dev` has 7.7 GiB and `generate()` for this unit does not fit in it:
#
#   python3 vit_harness.py YawRateControl ... --no-build   ->  RC=137, Killed
#   free -g inside vit-dev afterwards                      ->  7 total, 7 free
#
# The kill lands immediately after the `ADMISSIBLE BASELINE` line, i.e. inside
# `generate()` and long before anything this campaign changed is reached. It is
# a property of the unit -- 507 varied parameters, several of them arrays of
# 1024 and 3000 elements, held as Python objects for every case at once -- and
# the 8093-case corpus already in the tree was generated when it just fitted.
# A corpus that can be produced once and never again is not reproducible
# evidence, so the generation moved to the host (36 GiB) and stayed there.
#
# ONLY THE GENERATION MOVES. `--no-build` stops before the compiler, so the
# `test` binary, the Fortran objects it links and the run that produces the
# artifact are all still the container's. `scripts/harness.sh --no-generate` is
# what builds and runs afterwards, against the corpus this wrote.
#
# THE CONTROL THAT MAKES THE MOVE ADMISSIBLE is in
# evidence/YawRateControl/corpus_host_generation_control.txt: this script run
# with NO extra arguments, against the three-state baseline, reproduces the
# container-generated corpus BYTE FOR BYTE -- both the case file and the test
# source. Without that, a host-generated corpus and a container-generated one
# are two different instruments and the campaign's own artifacts would span
# both.
#
# The venv exists because the host python is externally managed and
# `vit_harness.py` imports PyYAML. It holds PyYAML and nothing else; `vit`
# itself comes off PYTHONPATH, from the same checkout the container mounts.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
WS="$(cd "$ROOT/.." && pwd)"
VENV="${VIT_HOST_VENV:-/tmp/yrcvenv}"

if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install --quiet pyyaml
fi

PYTHONPATH="$WS/vit" "$VENV/bin/python" "$WS/translation-loop/scripts/vit_harness.py" \
    YawRateControl --root "$ROOT" \
    --file rosco/controller/src/Controllers.f90 \
    --cpp translations/Controllers/yawratecontrol.cpp \
    --module Controllers --no-build "$@"
