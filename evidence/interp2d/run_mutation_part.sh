#!/bin/bash
# ONE operator-filtered mutation part for unit #45 `interp2d`, guarded.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/interp2d/run_mutation_part.sh <part-name> <operator>[,<operator>...] [--equivalences]
#
# WHY IT IS SPLIT BY OPERATOR. A dispatch's foreground command may block for 600
# seconds; unit #41 lost a 69-mutant part to exactly that ceiling. Each part
# blocks, and `scripts/_mutation_merge.py` unions them -- refusing a set that
# does not cover every operator `harness.cppmutate` offers, so the split cannot
# silently drop one.
#
# WHY IT MUST RUN ON THE CLEAN TREE. Twice over here. On an integrated tree the
# harness's Fortran side calls the wrapper, which calls `interp2d_c`, which
# links the harness's own compiled copy -- so both sides would be the mutant,
# and `vit_mutate.py` refuses that configuration outright. And this unit's
# CALLEE `interp1d` is itself integrated, so on an integrated tree the reference
# side's interp1d is the C++ one as well.
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
    EQ="--equivalences /workspace/ROSCO-r2/mutation/interp2d.equivalences.json"

OPARGS=""
IFS=, read -ra _ops <<< "$OPS"
for o in "${_ops[@]}"; do OPARGS="$OPARGS --operator $o"; done

bash scripts/mutate_guarded.sh translations/Functions/interp2d.cpp \
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 /workspace/translation-loop/scripts/vit_mutate.py \
         interp2d --root /workspace/ROSCO-r2 \
         --cpp translations/Functions/interp2d.cpp --module Functions \
         $OPARGS $EQ --out mutation/interp2d.clean.$PART.json"
