#!/bin/bash
# The ARM CENSUS for unit #44 `YawRateControl`: the shipped translation plus
# counters, run over a corpus, printing how many cases enter each arm.
#
#   bash scripts/run_if_time_remains.sh 900 \
#       bash evidence/YawRateControl/run_arm_census.sh [--no-generate] [extra vit_harness args...]
#
# WHY IT IS A SCRIPT. The first census (2026-08-15) was produced by hand and the
# steps lived in a session transcript. The second one had to establish that the
# corpus had CHANGED in one respect and in no other, which is a comparison
# against the first -- and a comparison against steps nobody wrote down is not
# one. `evidence/YawRateControl/arm_census.txt` is this script's output.
#
# The probe is `yawratecontrol.arm_census_probe.cpp`: the shipped translation
# with counters and an atexit dump added and NOTHING else changed. If the run
# reports HARNESS PASS the census is a READING of the corpus rather than a
# perturbation of it, and the pass/fail line is kept in the output for that
# reason.
#
# The EXIT trap restores the translation from git, for the reason every stub
# runner in this campaign has one: three of three hard kills have left an edit
# behind. COMMIT THE TRANSLATION FIRST.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Controllers/yawratecontrol.cpp
E=evidence/YawRateControl
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT
cp "$E/yawratecontrol.arm_census_probe.cpp" "$CPP"
bash scripts/harness.sh YawRateControl Controllers yawratecontrol \
    rosco/controller/src/Controllers.f90 \
    --out /tmp/yawratecontrol.census.json "$@" 2>&1 | tail -40
