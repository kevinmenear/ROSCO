#!/bin/bash
# Run one counting probe through the committed differential harness and keep the
# artifact. From the repository root.
#
#   bash evidence/PitchSaturation/run_probe.sh <probe.cpp> <out.json> "<what it counts>"
#
# Copied from evidence/PIIController/run_probe.sh (unit #35) with the unit,
# module and stem changed, and with its counting-channel paragraph dropped --
# this unit runs STUBS through the harness, not counting probes, so there is no
# sentinel channel to argue for. What it keeps is the part that matters: the
# artifact is written by `harness.sh --red-test`, which records the perturbation
# INTO the artifact, and the translation is restored from HEAD unconditionally.
#
# NOT `--no-generate`. In pre mode that flag prepares the build files and EXITS
# BEFORE running `./test` -- it exists for the mutation re-take, where
# `vit_mutate.py` runs `make` itself. The corpus is regenerated instead, and it
# is the same corpus: the generator mines the REFERENCE's literals and a stub
# changes only the translation. 1036 cases either way, which is what makes the
# red comparable to the green (unit #26's rule).
#
# `--disable R13_staging_capacity` because the green it red-tests was taken with
# R13 ablated -- see harness_generator_defect/R13_double_gate.txt. A red test on
# a different corpus from its green is not a red test of that green.
#
# The translation is restored from HEAD afterwards, always -- these probes are
# edits to the file `vit_mutate.py` mutates in place, and this campaign has lost
# three sessions to a mutant left in the tree.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PROBE="$1"; OUT="$2"; WHAT="$3"
CPP=translations/ControllerBlocks/pitchsaturation.cpp

cp "$PROBE" "$CPP"
bash scripts/harness.sh PitchSaturation ControllerBlocks pitchsaturation \
     rosco/controller/src/ControllerBlocks.f90 --against translation \
     --disable R13_staging_capacity \
     --out "$OUT" --red-test "$WHAT" > /tmp/probe.log 2>&1 || true
git checkout HEAD -- "$CPP"
python3 - "$OUT" "$PROBE" <<'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"{sys.argv[2]:<62} {d['failed']:>6} of {d['checked']}")
PYEOF
