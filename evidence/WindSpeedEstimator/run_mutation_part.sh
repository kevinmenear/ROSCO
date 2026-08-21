#!/bin/bash
# ONE operator-filtered mutation part for unit #65 `WindSpeedEstimator`, guarded.
#
#   bash scripts/run_if_time_remains.sh 600 \
#       bash evidence/WindSpeedEstimator/run_mutation_part.sh <part-name> <operator>[,<op>...] \
#            [--equivalences] [--unreachable] [--sanitize]
#
# Copied from evidence/AeroDynTorque/run_mutation_part.sh (unit #48), which
# copied it from interp2d (unit #45). Its three reasons hold here unchanged and
# are not restated; what IS different is the size.
#
# WHY IT IS SPLIT BY OPERATOR, AND WHY THIS UNIT HAD NO CHOICE. 278 mutants --
# the campaign's largest population -- with SIX operators at `cppmutate`'s
# 40-per-operator cap (arith_op, compare_op, const_tweak, drop_factor,
# index_offset) and 78 more across seven small ones. A dispatch's foreground
# command may block for 600 seconds; unit #41 lost a 69-mutant part to exactly
# that ceiling. `scripts/_mutation_merge.py` unions the parts and REFUSES a set
# that does not cover every operator `harness.cppmutate` offers, so the split
# cannot silently drop one.
#
# WHY IT MUST RUN ON THE CLEAN TREE. On an integrated tree the harness's Fortran
# side calls the wrapper, which calls `windspeedestimator_c`, which links the
# harness's own compiled copy -- both sides would be the mutant, and
# `vit_mutate.py` refuses that configuration outright. And all SIX of this
# unit's callees are themselves integrated, so on an integrated tree the
# reference side's saturate, LPFilter, AeroDynTorque, interp1d, interp2d and
# identity would be the C++ ones as well.
#
# WHY `mutate_guarded.sh` AND NOT A BARE CALL. `vit_mutate.py` edits the shipped
# .cpp IN PLACE and restores it on completion; killed, it does not, and three of
# three hard kills in this campaign have left a mutant in the tree.
#
# THE CORPUS IS THE ONE ON DISK, 13,868 cases, and it is the one
# `harness/WindSpeedEstimator.json` reported green on -- unit #26's rule.
# `vit_mutate.py` reuses `<stem>_cases.bin` rather than regenerating. Run
# `harness.sh ... --no-generate` after any reset and BEFORE the first part, or
# the link is the previous tree's (unit #46).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
PART="${1:?usage: run_mutation_part.sh <part-name> <operator>[,<op>...] [--equivalences] [--unreachable] [--sanitize]}"
OPS="${2:?}"
shift 2

EQ=""
UN=""
SAN=""
for a in "$@"; do
    case "$a" in
        --equivalences) EQ="--equivalences /workspace/ROSCO-r2/mutation/WindSpeedEstimator.equivalences.json" ;;
        --unreachable)  UN="--unreachable /workspace/ROSCO-r2/mutation/WindSpeedEstimator.unreachable.json" ;;
        --sanitize)     SAN="--sanitize" ;;
        *) echo "run_mutation_part: unknown flag $a" >&2; exit 2 ;;
    esac
done

OPARGS=""
IFS=, read -ra _ops <<< "$OPS"
for o in "${_ops[@]}"; do OPARGS="$OPARGS --operator $o"; done

bash scripts/mutate_guarded.sh translations/ControllerBlocks/windspeedestimator.cpp \
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 /workspace/translation-loop/scripts/vit_mutate.py \
         WindSpeedEstimator --root /workspace/ROSCO-r2 \
         --cpp translations/ControllerBlocks/windspeedestimator.cpp --module ControllerBlocks \
         $OPARGS $EQ $UN $SAN --out mutation/WindSpeedEstimator.clean.$PART.json"
