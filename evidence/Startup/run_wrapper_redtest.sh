#!/bin/bash
# The POST-INTEGRATION red test: perturb the generated WRAPPER, not the C++.
#
#   bash scripts/run_if_time_remains.sh 300 bash evidence/Startup/run_wrapper_redtest.sh
#
# `--post-integration` measures the MARSHALLING and nothing else: after
# integration the Fortran body IS the translation, so both sides of the
# comparison run one implementation by construction and a green says only that
# the view populate / copy-back round trip preserved every argument. What can
# fail there is the copy-back, which is why this unit needed `--reverse-copy`:
# all five of its outputs -- SU_Stage, SU_LoadStageStartTime, SU_RotSpeedF,
# PRC_R_Speed and PRC_R_Torque -- are SCALAR fields of an INOUT view argument
# and do not travel home through a C_LOC'd buffer the way an allocatable array
# does.
#
# So the perturbation is the deletion of exactly that line, scoped to THIS
# unit's wrapper by evidence/Startup/scope_wrapper_perturb.py -- the line
# appears six times in the file and a whole-file replace measures six wrappers.
# Read that script's header; the scoping is the finding, and it is unit #41's.
#
# TWO THINGS THE TRAP AND THE `touch` ARE FOR, both measured in this campaign:
#   * the EXIT trap restores the source from git, because a killed dispatch that
#     does not leaves a perturbed wrapper in a tracked file;
#   * the `touch` after the restore is the bind-mount mtime hazard (RUNBOOK,
#     unit #38): `git checkout` can write a file whose mtime make treats as
#     older than the object, so the "restored" build is still the perturbed one
#     and a re-taken green would be a lie. Re-take the green; do not infer it.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/ControllerBlocks.f90
build () {
    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller/build && \
        cmake --build . -j8 >/dev/null 2>&1 && \
        cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
}
restore () { git checkout -- "$F"; touch "$F"; build; echo "restored $F, rebuilt and installed"; }
trap restore EXIT

python3.12 evidence/Startup/scope_wrapper_perturb.py "$F" || exit 2
touch "$F"; build
bash scripts/harness.sh Startup ControllerBlocks startup "$F" \
    --post-integration --out evidence/Startup/harness.postintegration.redtest.json \
    --red-test "Startup's wrapper: CALL vit_copy_scalars_to_localvariables deleted -- the five scalar outputs never leave the view" 2>&1 | tail -8
