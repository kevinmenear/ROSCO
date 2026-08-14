# `Debug` — the mutation score, and what the 57 survivors are

`mutation/Debug.json`: **121 of 178 behavioural killed, 0.6798**, 3 no-compile
of 181, 0 declared equivalent, 10 operators. Below the campaign's 1.000
threshold, so the unit closes `deferred`.

## The instrument, because it is not `vit_mutate.py`

`vit_mutate.py` scores a mutant by rebuilding the generated differential harness
and comparing the unit's **mapped outputs**. `Debug` assigns nothing in its own
signature, so every mutant would survive and the 0.000 would be a fact about the
instrument. `scripts/dbgmutate.py` uses the oracle that verified the unit
instead: each mutant is compiled into `libdiscon.so`, scenarios 27 and 28 are
run, and the `.RO.dbg*` bytes are compared against `preall` — a committed
archive written by a build containing no C++ `Debug` at all.

**The reference side cannot be the mutant**, which is the configuration question
unit #29 got wrong. `preall` is bytes on disk, fixed before the sweep starts;
nothing the sweep does can perturb it.

The sweep is nine foreground parts (`--operator`, and `--slice` for the three
operators that produce 40 each), merged by `scripts/dbgmutate_merge.py`, which
asks `harness.cppmutate` for the operator population directly and refuses a
union that does not exhaust it.

**It refused once, correctly.** The first union named three different
`loop_rev`s, because two mutator defects were fixed mid-sweep. Six parts were
re-taken so that all nine name `897d67c`.

## Three mutator defects were fixed before this number existed (X2)

All three in `harness/cppmutate.py`, in the repo that owns it, each with its
cost measured across all 31 translations rather than argued.

| defect | seen as | cost |
|---|---|---|
| angle brackets of a **user-declared template** rewritten by `compare_op` (`template <=int W>`) | 12 of 40 compare_op mutants unbuildable — 30%, above the 25% at which a run refuses | 1 unit, 3 ids |
| `arith_op` reading a **qualified pointer declaration**, a **dereference after a keyword**, and a **float exponent sign** as arithmetic (`std::FILE / f`, `return / p`, `1E + 99`) | 13 of 25 unbuildable — 52% | 2 units, 17 ids, 0 gained |
| `_not_arithmetic` reading **`drop_factor`'s right operand as its operator**, so it fired for two rules of three; and `swap_operands` moving a call's name out of its parentheses (`buf.size - pos()`) | 7 of 22 unbuildable | 2 units, 11 ids, 0 gained |

The third was found by running the sweep, not by reading the diff — which is the
argument for scoring every operator rather than the interesting ones.

The `1E-99` case is the one that mattered beyond the ratio: it meant the two
clamp constants of this unit had **no `arith_op` mutant at all**, so an operator
that appeared to be measuring them was measuring nothing there.

## The 57 survivors, by family

None is a scatter. Every one is a shape, and each shape names what would close
it.

### 1 — 11 survivors: a buffer or array extent made LARGER

`char c[W]` → `c[W + 1]`, `char tmp[512]` → `[513]`, `char date[11]` → `[12]`,
`DebugOutStrings_src[26]` → `[27]`, `512` → `513`, `11` → `12`, `8` → `9`.

Over-allocation. No comparison of written bytes can see a buffer that is one
element longer than it needs to be. Read_OL_Input's family (1) with the sign
flipped — there the mutants read one byte PAST a buffer, which is undefined
behaviour no value comparison can see; here they never read it at all.

### 2 — 7 survivors: the STDOUT status line

`std::fmod(Time, 10.0) == 0.0` (compare_op, negate_cond, drop_call,
swap_call_args, swap_callee → `remainder`), `GenSpeedF * RPS2RPM` (arith_op,
drop_factor), and the `RPS2RPM` / `R2D` constants themselves.

`WRITE(*, 100)` writes to unit 6. The oracle reads FILES. This is the one
blind spot with an obvious remedy — capture and compare each scenario's stdout —
and it is not taken here.

### 3 — 7 survivors: the two clamps, at their boundary

`fabs(x) < 1E-99` → `<=`, `fabs(x) > 1E+99` → `>=`, and the `fabs` dropped from
the upper test. The `<`/`<=` pair differs at exactly `|x| == 1E-99`; nothing the
controller computes lands on it. The upper clamp is **dead outright** — measured
separately at 0 of 408,072 records (`dbg_redtest.txt`), while its sibling one
line up moves 21,792.

### 4 — 5 survivors: guards whose operand is constant in every input

`LoggingLevel > 0` → `>= 0`, `> 1` → `>= 1`, `> 2` → `>= 2`, `CC_Mode > 0` →
`>= 0`, `StC_Mode > 0` → `>= 0`. `LoggingLevel` is 1 in the 14 shipped inputs and
3 in scenario 28; the pairs differ only at 0 and at 2. `CC_Mode` and `StC_Mode`
are 0 or 1. Closed by an input at the missing value, nothing else.

### 5 — 3 survivors: an extent of ONE

`CC_GroupIndex[Ind - 1]` → `[1 - Ind]`, twice, and the same for
`StC_GroupIndex`. Scenario 28 sets `CC_Group_N = 1` and `StC_Group_N = 1`, so
`Ind` is always 1 and `Ind - 1 == 1 - Ind == 0`. Equivalent AT EXTENT 1 and
killed by any input with a group count of 2.

### 6 — 8 survivors: arms of the format helpers this unit cannot reach

`len >= w` → `>` (a heading literal exactly as wide as its field: never — the
widths are 15/4/5 against 20 and 14/3 against 21), `w >= need` (the `Infinity`
spelling; no output is infinite after the clamp), `edig.size() > 1` (C's `%E`
emits exactly two exponent digits, so the leading-zero strip runs at most once),
`std::string(w + 1, '*')` (the asterisk overflow arm), `CFI_CDESC_T(1)` → `(2)`.

Two of these are genuinely **equivalent** rather than unreached and are NOT
declared as such here, because declaring an equivalence is a claim this dispatch
did not have time to prove: `pos > buf.size()` → `>=` (the extra call is
`append(0, ' ')`, a no-op) and `if (UnDb)` → `if (!(UnDb))` on the close block
(stdio flushes every open stream at process exit, so the bytes are identical).
The score is reported with them counted against it.

### 7 — 3 survivors: `TRIM(RootName)`

`n > 0` → `>= 0`, `n - 1` → `n + 1` / `1 - n`, `0` → `1`. Every RootName the
scenarios pass is already blank-free, so the trim loop body never executes.

### 8 — 13 survivors: the index shifts, AND THE CORPUS IS WHY

`DebugOutData[11]`, `[12]`, `[13]`, `[21]` → `[+1]`, and the rest of the
`index_offset` set.

**This one was measured rather than reasoned about.** The `[11] → [12]` shift
was run over the FULL 24-file corpus, through the same red-test path as every
other perturbation:

    23,998 of 408,072 records, in 1 of 24 files -- vit_sim7.RO.dbg

`DebugVar%NacIMU_FA_AccF` and `DebugVar%FA_AccF` are both zero in scenarios 27
and 28 (1-DOF sims where the nacelle IMU never moves), so writing one into the
other's slot changes nothing there. Scenario 7 drives them.

**So the score is a statement about the corpus {27, 28}, and a wider one is
known to raise it.** That is this unit's largest open gap and it is cheap to
close: `--scenarios 7,27,28` costs about 50% more wall clock per mutant.
Artifact: `evidence/Debug/dbg.redtest.survivor_idx11.json`.

## Why {27, 28} and not the 24 files

A mutant here costs a rebuild plus a full simulation. Scenario 28 is scenario
3's configuration at `LoggingLevel = 3`, so it is the only input that reaches
the `.RO.dbg2` / `.RO.dbg3` half of the unit at all — a third of the body — and
it subsumes scenario 3. Scenario 27 adds a second set of dynamics. At ~11
seconds per mutant, 181 mutants is 33 minutes, which is already six foreground
commands.

The ablation is committed beside the score:
`mutation/Debug.compare_op.no28.{0,1}.json` is the same 40 compare_op mutants
against scenarios 3, 7 and 27 — LoggingLevel = 1, like every shipped input —
and kills **12 of 40** where {27, 28} kills **21 of 40**. Adding one input
parameter nearly doubled that operator's kill count.
