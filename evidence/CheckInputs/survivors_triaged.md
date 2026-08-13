# The 89 survivors, one bucket at a time

**Date:** 2026-08-13
**Status:** in-progress — round 1 of the input repair is measured, round 2 is designed and measured on one survivor
**Scope:** unit #29, P12 / E4.6

`mutation/CheckInputs.json` after the R11 re-take: **84 killed of 173, 0.4855**,
up from 8 of 173. This is the triage of what is left, in the three categories
the dispatch names:

* **(a) equivalent** — the mutated program agrees with the original on every
  admissible input;
* **(b) the harness cannot reach it** — fix the inputs, not the record;
* **(c) a blind spot no rule covers** — a new category, escalated.

Placement is `evidence/CheckInputs/survivor_sites.py`, which recomputes each id
from `harness.cppmutate` over the committed translation, so a survivor is
attributed to a line rather than to a token.

| bucket | n | verdict |
|---|---|---|
| A. the two named constants | 2 | (b) — needs TWO quantities off baseline at once |
| B. the message sink's own guards | 5 | (c) — the EMITTER pads every staging buffer; no case can reach the capacity guard |
| C. the array-intrinsic helpers | 26 | (b) — R11's arrays are constant, measured |
| D. the AWC / CC / StC / OL loops | 23 | (b) for 21, and 2 pinned by an out-of-bounds READ in the reference |
| E. the checks themselves | 33 | (b) for 30, (a) for 1, 2 open |
| | **89** | |

---

## C and D — R11's OWN ARRAYS ARE CONSTANT, WHICH IS THE DEGENERACY R5 EXISTS TO REMOVE

49 of the 89, and the largest single cause. Measured, not argued, in
`survivor_5769f7da_broadcast.txt`:

```
                                 reference                       the mutant [0] -> [0+1]
broadcast    {fill: -1.0}        F_FlCornerFreq(2) ...           F_FlCornerFreq(2) ...      IDENTICAL
one_element  {fill: 1.0, 0:-1}   F_FlCornerFreq(1) ...           aviFAIL 0, no message      KILLED
```

Two mechanisms, both about the SHAPE of R11's arrays rather than its values:

* the inert fill is **zeros**, so every unnamed array is constant;
* a whole-array knob **broadcasts**, so the perturbed array is constant too.

An index shift, a loop-bound shift and a `<`/`<=` on a loop bound all move
inside a run of identical values, and every one of them lands on the same
answer. R5 already says "array elements distinct" about the corpus; R11's own
block did not have that property. R7 broadcasts for a reason it states — one
element set leaves the FALSE side at `_fill_array`'s ramp, which straddles the
tested literal — and **that reason does not hold in R11, whose other elements
are admissible by construction**.

Repair (round 2): a `ramp` form in the baseline so an array is admissible AND
distinct, and one-element placement at 1 / interior / n for a whole-array knob,
beside the broadcast rather than instead of it.

Two of D are not this: `CC_GroupIndex[I-1]` and `StC_GroupIndex[I-1]` sit in
loops whose trip count is PINNED in `harness/ranges.toml` (`CC_Group_N`,
`StC_Group_N` at [0, 2]) because **the reference reads past the end of the array
without checking** — case 9544 killed the Fortran with `AWC_NumModes = 99999`
against 28 elements. Widening the pin is not available: it does not test the
translation, it invokes the reference's own undefined behaviour. These are (b)
with the reason stated and no repair, which is the honest form of (b).

## E — the checks

30 of the 33 are (b) and each names its own repair:

* **the ZMQ family** (5: `1b723864`, `44bc2497`, `8f7572d1`, `a5a8c2e8`,
  `9354f6d8`, and `36317c82`/`43d4fe83`/`1042577b` on the guard) — the check is
  `ABS(ZMQ_UpdatePeriod - DT*n_DT_ZMQ) > 0.001`, and R11's inert fill leaves
  **both `ZMQ_UpdatePeriod` and `n_DT_ZMQ` at zero**, where `a - b` and `a + b`
  agree, `fabs` is the identity and every constant tweak is below the threshold.
  This is R3's dead-signal rule — "no real may be zero in every case" —
  reappearing inside a rule that was written to remove masking. Repair:
  admissible NON-ZERO values (`ZMQ_UpdatePeriod = 0.05`, `n_DT_ZMQ = 5`, with
  `DT = 0.01`, which satisfies the check exactly).
* **the array-element checks** (13: `F_FlCornerFreq`, `F_FlpCornerFreq`,
  `IPC_KI` x4, `VS_KP`, `VS_KI`, `PerfTableSize` x2, `Y_ErrThresh`,
  `avrSWAP[9]`, `Ind_BldPitch`) — the same constancy as C and D, in the checks
  rather than in the helpers. Same repair.
* **`F_LPFType > 2` -> `>= 2`** (`c7770aac`) — reachable in principle:
  `F_LPFType = 2` is admissible and makes the mutant fire where the reference
  does not. It survives because a state that sets `F_LPFType = 2` trips the
  LATER `F_LPFDamping` check as well, and the later message wins. Repair:
  carry an admissible `F_LPFDamping > 0` in the baseline, so the second check
  is already satisfied when the knob moves the first.
* **the `VS_FBP > 0 .AND. PC_ControlMode > 0` conjunction** (`a1f04734`,
  `dc787a64`) — a predicate over two quantities, and R11 moves one. The
  `open_loop_flap_fbp` state already holds `VS_FBP = 1`, so the knob on
  `PC_ControlMode` should reach it; it did not, which means a later check fires
  under `PC_ControlMode > 0` too. OPEN — the two that round 2 has to look at
  rather than assume.
* **`IPC_ControlMode > 2` -> `> 3`** (`8ea1df39`), **`DT_Out` compare_ops**
  (`5e7c786f`, `6f6c1760`, `178cca7e`), **`316503f1`** — reachable with a value
  that trips exactly one of a nested family; `DT_Out = 0.0005` with `DT = 1`
  and `n_DT_Out = 0` isolates the second `DT_Out` check, for instance. Repair:
  states, not new rules.

**One is (a), and it is provable.** `9b706bcb`, `const_tweak '1' -> '2'` at
line 168 — `ErrVar->aviFAIL = -1` inside `IF (DT_Out <= 0)`, making it `-2`:

> `DT_Out <= 0` implies either `DT_Out < DT` (the very next check, which assigns
> `aviFAIL = -1`) or `DT >= DT_Out`, i.e. `DT <= 0`, which trips
> "DT must be greater than zero" further down, which also assigns `aviFAIL = -1`.
> There is no early return, so whenever the mutated statement executes, a LATER
> statement overwrites both of its outputs. The two programs agree on every
> input.

That is a dominance argument about the reference's own check ORDER, not about
the corpus, and it is the shape that makes a 1.0 threshold unattainable for this
unit: **a mutant on a dominated write is equivalent at the interface**, and no
corpus can kill it. It is declared here rather than in
`mutation/CheckInputs.equivalences.json` so that it can be disputed before it
changes a score.

## B — (c), AND IT BELONGS TO THE EMITTER RATHER THAN TO ANY CORPUS RULE

Five mutants sit in the translation's message sink: the two capacity guards
(`s.size() > cap`, `cap > s.size()`), the `n <= 0` guard in `trim`, and the
`find_last_not_of(' ') + 1`. `harness/emit.py` sizes every deferred-length
staging buffer as **`n + 4096`**, so `s.size() == cap` is unreachable by
construction — no case in any corpus this emitter writes can make the guard
change its answer. Widening the baseline cannot touch it; the buffer size is
the instrument's, not the unit's.

Recorded as a known gap rather than declared equivalent, because the guards are
NOT dead in the shipped program: there the capacity is the integration
wrapper's staging buffer and a message can be exactly as long as it.

## A — two constants, and R11's structural limit

`VS_Mode_WSE_TSR` and `VS_Mode_Power_TSR` are compared against `VS_ControlMode`
inside `IF (TRA_Mode > 1)`. Reaching them needs `TRA_Mode > 1` **and** a
particular `VS_ControlMode`: two quantities away from their baseline value, and
R11 moves exactly one, on purpose. The repair is not to relax that — it is to
add a state with `TRA_Mode = 2`, after which one knob is enough. **A state is
how R11 reaches a conjunction**, and that is the general answer to this bucket.
