#!/bin/bash
# ONE operator-filtered mutation part for unit #48 `AeroDynTorque`, guarded.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/AeroDynTorque/run_mutation_part.sh <part-name> <operator>[,<operator>...] [--equivalences]
#
# Copied from evidence/interp2d/run_mutation_part.sh (unit #45) with the unit,
# the module and the path changed. Its three reasons hold here unchanged:
#
# WHY IT IS SPLIT BY OPERATOR. A dispatch's foreground command may block for 600
# seconds; unit #41 lost a 69-mutant part to exactly that ceiling. This unit has
# only 35 mutants and one run would probably fit -- "probably" is the reason it
# is split anyway. Each part blocks, and `scripts/_mutation_merge.py` unions
# them, refusing a set that does not cover every operator `harness.cppmutate`
# offers, so the split cannot silently drop one.
#
# WHY IT MUST RUN ON THE CLEAN TREE. Twice over here, exactly as for interp2d.
# On an integrated tree the harness's Fortran side calls the wrapper, which
# calls `aerodyntorque_c`, which links the harness's own compiled copy -- so
# both sides would be the mutant, and `vit_mutate.py` refuses that configuration
# outright. And this unit's CALLEE `interp2d` is itself integrated, so on an
# integrated tree the reference side's interp2d is the C++ one as well.
#
# WHY `mutate_guarded.sh` AND NOT A BARE CALL. `vit_mutate.py` edits the shipped
# .cpp IN PLACE and restores it on completion; killed, it does not, and three of
# three hard kills in this campaign have left a mutant in the tree. The guard
# raises a marker that only comes down when the file hashes back.
#
# THE CORPUS IS THE ONE ON DISK AND IT IS NO LONGER THE ABLATED ONE (second
# dispatch). `vit_mutate.py` reuses `<stem>_cases.bin` rather than regenerating,
# so the score below is a statement about the 1373-case corpus
# `harness/AeroDynTorque.json` was green on -- which is unit #26's rule.
# `--disable R13_staging_capacity` is gone; what remains is a STATED HOLE of
# fourteen capacities in `harness/ranges.toml`, [16, 29], which is where the two
# chains gate the same buffer a different number of times and the case has no
# oracle. That is 242 capacities MORE than the ablation carried, including the
# one at which the refusal-boundary mutant `88466711` dies (30). The reason for
# the hole is in evidence/AeroDynTorque/harness.staging_composition.txt.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
PART="${1:?usage: run_mutation_part.sh <part-name> <operator>[,<op>...] [--equivalences]}"
OPS="${2:?}"
EQ=""
[ "${3:-}" = "--equivalences" ] && \
    EQ="--equivalences /workspace/ROSCO-r2/mutation/AeroDynTorque.equivalences.json"

OPARGS=""
IFS=, read -ra _ops <<< "$OPS"
for o in "${_ops[@]}"; do OPARGS="$OPARGS --operator $o"; done

bash scripts/mutate_guarded.sh translations/Functions/aerodyntorque.cpp \
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 /workspace/translation-loop/scripts/vit_mutate.py \
         AeroDynTorque --root /workspace/ROSCO-r2 \
         --cpp translations/Functions/aerodyntorque.cpp --module Functions \
         $OPARGS $EQ --out mutation/AeroDynTorque.clean.$PART.json"
