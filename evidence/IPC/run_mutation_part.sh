#!/bin/bash
# ONE operator-filtered mutation part for unit #53 `IPC`, guarded.
#
#   bash scripts/run_if_time_remains.sh 900 \
#       bash evidence/IPC/run_mutation_part.sh <part-name> <operator>[,<operator>...] [--equivalences]
#
# Copied from evidence/ForeAftDamping/run_mutation_part.sh (unit #52) with the
# unit, the stem and the callee list changed. Its three reasons hold unchanged:
#
# WHY IT IS SPLIT BY OPERATOR. A dispatch's foreground command may block for 600
# seconds; unit #41 lost a 69-mutant part to exactly that ceiling. Each part
# blocks, and `scripts/_mutation_merge.py` unions them, refusing a set that does
# not cover every operator `harness.cppmutate` offers, so the split cannot
# silently drop one.
#
# AND THIS UNIT HAS TO BE SPLIT -- IT IS THE OPPOSITE OF #52's CASE. Counted
# before the sweep with `cppmutate.mutants('IPC', <the .cpp>)`:
#
#     const_tweak 40   index_offset 18   arith_op 17   swap_operands 15
#     compare_op  13   negate_cond   9   drop_call  2   swap_call_args 2
#     swap_callee  2                                      TOTAL 118
#
# 118 against unit #52's 9, on a corpus that runs six callees per case. An
# unfiltered run would be the shape unit #41 lost.
#
# WHY IT MUST RUN ON THE CLEAN TREE. Twice over. On an integrated tree the
# harness's Fortran side calls the wrapper, which calls `ipc_c`, which links the
# harness's own compiled copy -- so both sides would be the mutant, and
# `vit_mutate.py` refuses that configuration outright. And ALL SIX of this
# unit's callees are themselves integrated, so on an integrated tree the
# reference side's ColemanTransform, ColemanTransformInverse, LPFilter,
# PIController, sigma and wrap_360 are the C++ ones as well.
#
# WHY `mutate_guarded.sh` AND NOT A BARE CALL. `vit_mutate.py` edits the shipped
# .cpp IN PLACE and restores it on completion; killed, it does not, and three of
# three hard kills in this campaign have left a mutant in the tree. The guard
# raises a marker that only comes down when the file hashes back.
#
# THE CORPUS IS THE ONE ON DISK. `vit_mutate.py` reuses `<stem>_cases.bin`
# rather than regenerating, so the score is a statement about the same corpus
# `harness/IPC.json` was green on -- which is unit #26's rule.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
PART="${1:?usage: run_mutation_part.sh <part-name> <operator>[,<op>...] [--equivalences] [--offset N] [--limit N]}"
OPS="${2:?}"
shift 2

# AND THE SLICE, WHICH IS WHY THIS UNIT CAN BE SCORED AT ALL. `const_tweak` is
# 40 mutants here at 26.5s each -- 1,040s against the 600s a foreground command
# may block for -- and until loop 4751161 `--limit` could only truncate, so
# mutants 21..40 were not addressable by any sequence of runs. `--offset N
# --limit M` names a window; `scripts/_mutation_merge.py` checks that the
# windows of the parts sharing an operator PARTITION it, by mutant id.
#
# Anything not recognised here is passed through to `vit_mutate.py` unchanged,
# so the two slice flags need no special case.
# `--unreachable` IS A DIFFERENT CLAIM FROM `--equivalences` AND IS SPELT
# SEPARATELY SO IT READS AS ONE. Added at the fourth dispatch, when the
# disposition became reachable from a unit session (loop b9c8c52). Equivalence
# says the two PROGRAMS agree on every admissible input; unreachable says this
# CORPUS never reaches the mutant -- a weaker claim, which is why
# `vit_mutate.load_unreachable` refuses any entry without a `reason` and an
# `evidence` path that EXISTS, and why `loop/done.py` P12 fails the unit
# outright if the corpus then kills one.
EQ=""
UNR=""
EXTRA=""
while [ $# -gt 0 ]; do
    case "$1" in
        --equivalences)
            EQ="--equivalences /workspace/ROSCO-r2/mutation/IPC.equivalences.json" ;;
        --unreachable)
            UNR="--unreachable /workspace/ROSCO-r2/mutation/IPC.unreachable.json" ;;
        *) EXTRA="$EXTRA $1" ;;
    esac
    shift
done

OPARGS=""
if [ "$OPS" != "all" ]; then
    IFS=, read -ra _ops <<< "$OPS"
    for o in "${_ops[@]}"; do OPARGS="$OPARGS --operator $o"; done
fi

bash scripts/mutate_guarded.sh translations/Controllers/ipc.cpp \
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 /workspace/translation-loop/scripts/vit_mutate.py \
         IPC --root /workspace/ROSCO-r2 \
         --cpp translations/Controllers/ipc.cpp --module Controllers \
         $OPARGS $EQ $UNR $EXTRA --out mutation/IPC.clean.$PART.json"
