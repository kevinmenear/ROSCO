#!/bin/bash
# ONE operator-filtered mutation part for unit #44 `YawRateControl`, guarded.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/YawRateControl/run_mutation_part.sh <part-name> <operator>[,<operator>...] [--equivalences]
#
# e.g.  ... run_mutation_part.sh compare_op compare_op
#       ... run_mutation_part.sh cond_swap negate_cond,swap_operands
#       ... run_mutation_part.sh const_tweak const_tweak --equivalences
#
# WHY IT IS SPLIT BY OPERATOR. A dispatch's foreground command may block for 600
# seconds and this unit's full sweep is longer than that; unit #41 lost a
# 69-mutant part to exactly that ceiling. Each part blocks, and
# `scripts/_mutation_merge.py` unions them -- refusing a set that does not cover
# every operator `harness.cppmutate` offers, so the split cannot silently drop
# one.
#
# WHY IT MUST RUN ON THE CLEAN TREE. On an integrated tree the harness's Fortran
# side calls the wrapper, which calls `yawratecontrol_c`, which links the
# harness's own compiled copy -- the mutant. Both sides would be the mutant.
# `vit_mutate.py` refuses that configuration outright; run
# `scripts/reset_to_clean.sh` and `scripts/harness.sh ... --no-generate` first,
# and `scripts/restore_integrated.sh` after the last part.
#
# WHY `mutate_guarded.sh` AND NOT A BARE CALL. `vit_mutate.py` edits the shipped
# .cpp IN PLACE and restores it on completion; killed, it does not, and three of
# three hard kills in this campaign have left a mutant in the tree. The guard
# raises a marker that only comes down when the file hashes back.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
PART="${1:?usage: run_mutation_part.sh <part-name> <operator>[,<op>...] [--equivalences]}"
OPS="${2:?}"
EQ=""
[ "${3:-}" = "--equivalences" ] && \
    EQ="--equivalences /workspace/ROSCO-r2/mutation/YawRateControl.equivalences.json"

OPARGS=""
IFS=, read -ra _ops <<< "$OPS"
for o in "${_ops[@]}"; do OPARGS="$OPARGS --operator $o"; done

bash scripts/mutate_guarded.sh translations/Controllers/yawratecontrol.cpp \
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 /workspace/translation-loop/scripts/vit_mutate.py \
         YawRateControl --root /workspace/ROSCO-r2 \
         --cpp translations/Controllers/yawratecontrol.cpp --module Controllers \
         $OPARGS $EQ --out mutation/YawRateControl.clean.$PART.json"
