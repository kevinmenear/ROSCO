#!/bin/bash
# The POST-INTEGRATION red test: perturb the generated WRAPPER, not the C++.
#
#   bash scripts/run_if_time_remains.sh 400 bash evidence/interp2d/run_wrapper_redtest.sh
#
# `--post-integration` measures the MARSHALLING and nothing else: after
# integration the Fortran body IS the translation, so both sides of the
# comparison run one implementation by construction and a green says only that
# the view populate / copy-back round trip preserved every argument.
#
# The perturbation is the deletion of this unit's ErrorVariables scalar
# copy-back, scoped to THIS unit's wrapper by
# evidence/interp2d/scope_wrapper_perturb.py -- the line stands four times in
# Functions.f90 and a whole-file replace measures four wrappers. Read that
# script's header; the scoping is unit #41's finding.
#
# WHAT IT REPRODUCES: the wrapper `vit integrate --apply` emits WITHOUT
# `--reverse-copy`. Unit #23 measured that one level down, on interp1d: the
# kernel passed it 62 of 62 and the gate passed it 5,252,000 of 5,252,000, and
# the post-integration harness was the only layer that could see it.
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
F=rosco/controller/src/Functions.f90
build () {
    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller/build && \
        cmake --build . -j8 >/dev/null 2>&1 && \
        cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
}
restore () { git checkout -- "$F"; touch "$F"; build; echo "restored $F, rebuilt and installed"; }
trap restore EXIT

python3.12 evidence/interp2d/scope_wrapper_perturb.py "$F" || exit 2
touch "$F"; build
bash scripts/harness.sh interp2d Functions interp2d "$F" \
    --post-integration --out evidence/interp2d/harness.postintegration.redtest.json \
    --red-test "interp2d's wrapper: CALL vit_copy_scalars_to_errorvariables deleted -- the wrapper vit integrate emits without --reverse-copy, so every write to ErrVar%aviFAIL and ErrVar%ErrMsg is lost" 2>&1 | tail -6
