#!/bin/bash
# ONE operator-filtered mutation part for unit #52 `ForeAftDamping`, guarded.
#
#   bash scripts/run_if_time_remains.sh 900 \
#       bash evidence/ForeAftDamping/run_mutation_part.sh <part-name> <operator>[,<operator>...] [--equivalences]
#
# Copied from evidence/FloatingFeedback/run_mutation_part.sh (unit #51) with the
# unit, the stem and the callee list changed. Its three reasons hold unchanged:
#
# WHY IT IS SPLIT BY OPERATOR. A dispatch's foreground command may block for 600
# seconds; unit #41 lost a 69-mutant part to exactly that ceiling. Each part
# blocks, and `scripts/_mutation_merge.py` unions them, refusing a set that does
# not cover every operator `harness.cppmutate` offers, so the split cannot
# silently drop one.
#
# WHY IT MUST RUN ON THE CLEAN TREE. Twice over. On an integrated tree the
# harness's Fortran side calls the wrapper, which calls `foreaftdamping_c`, which
# links the harness's own compiled copy -- so both sides would be the mutant, and
# `vit_mutate.py` refuses that configuration outright. And this unit's CALLEE,
# PIController, is itself integrated, so on an integrated tree the reference
# side's PIController is the C++ one as well.
#
# WHY `mutate_guarded.sh` AND NOT A BARE CALL. `vit_mutate.py` edits the shipped
# .cpp IN PLACE and restores it on completion; killed, it does not, and three of
# three hard kills in this campaign have left a mutant in the tree. The guard
# raises a marker that only comes down when the file hashes back.
#
# THE CORPUS IS THE ONE ON DISK. `vit_mutate.py` reuses `<stem>_cases.bin`
# rather than regenerating, so the score is a statement about the same corpus
# `harness/ForeAftDamping.json` was green on -- which is unit #26's rule.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
PART="${1:?usage: run_mutation_part.sh <part-name> <operator>[,<op>...] [--equivalences]}"
OPS="${2:?}"
EQ=""
[ "${3:-}" = "--equivalences" ] && \
    EQ="--equivalences /workspace/ROSCO-r2/mutation/ForeAftDamping.equivalences.json"

OPARGS=""
IFS=, read -ra _ops <<< "$OPS"
for o in "${_ops[@]}"; do OPARGS="$OPARGS --operator $o"; done

bash scripts/mutate_guarded.sh translations/Controllers/foreaftdamping.cpp \
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 /workspace/translation-loop/scripts/vit_mutate.py \
         ForeAftDamping --root /workspace/ROSCO-r2 \
         --cpp translations/Controllers/foreaftdamping.cpp --module Controllers \
         $OPARGS $EQ --out mutation/ForeAftDamping.clean.$PART.json"
