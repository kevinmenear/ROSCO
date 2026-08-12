# StateMachine — unit #20

`SUBROUTINE StateMachine(CntrPar, LocalVar)` in
`rosco/controller/src/ControllerBlocks.f90`. The turbine's operating-state
decision tree: it reads nine `LocalVar` fields and six `CntrPar` fields and
writes exactly two, `LocalVar%PC_State` and `LocalVar%VS_State`. There is no
arithmetic in it at all — every statement is a predicate or an assignment of a
named `Constants.f90` PARAMETER — so this is the first unit in this campaign
whose entire content is control flow, and the corpus rules that matter are the
ones that decide which branch a case takes.

**Five layers ran and all five are alive.** The fifth unit of which that is
true, after `LPFilter` (#11), `NotchFilter` (#13), `SecLPFilter` (#18) and
`SecLPFilter_Vel` (#19).

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62 cases, **14,260 compared field rows, ALL `IDENTICAL`** | wrong-constant stub **0 of 62**, moves 123 rows |
| differential harness (clean Fortran) | **3610 checked, 0 failed, 0 inadmissible** | the unit as a no-op fails **3610 of 3610**, naming both outputs |
| mutation score | **39 of 39** behavioural killed, **1.000**, **0 declared equivalent**, 0 no-compile | — |
| post-integration harness (the wrapper) | 3610 checked, 0 failed | the reverse-copy removed: **3610 of 3610** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 36,577 of 5,252,000**, 0 after revert |

## The no-op is a MIRROR here, and it passes 60 of 62 kernel cases

Both outputs are fields of the `INTENT(INOUT)` argument, so they arrive
carrying **the previous timestep's answer**. A state machine's whole job is to
hold its state, so on any call where the state does not change, "write nothing"
and "write the right answer" are the same bytes. Measured, in
`kernel_field_rows.txt`:

```
no-op stub:            60 of 62 cases PASSED, 3 rows move (cases 1 and 2 only)
right-constant stub:   61 of 62 cases PASSED, 2 rows move (case 1 only)
wrong-constant stub:    0 of 62 cases PASSED, 123 rows move (all 62 cases)
```

This is unit #7's aliasing shape — where `CALL GetRoot(RootName,RootName)`
made the input and the reference output the same bytes — reached from a
different direction: **not two dummies aliasing one variable, but one field
aliasing its own previous value through time.** Any unit whose output is a
`LocalVar` field it also reads is this shape, and there are many of them left.
So the liveness test is the WRONG-CONSTANT stub, and it is the only one of the
three that says anything: `VS_State = 7` (`VS_State_PI`, a value no branch of
this unit can produce) fails every case.

What the other two say is how little the window constrains. **61 of the 62
captured cases have the same answer**, `PC_State = 1, VS_State = 1`, so a stub
reading no argument and writing those two constants passes 61 of 62 — the
kernel is a lookup table on all but the initialisation case. `GenArTq` and
`GenBrTq` are **0.0 in all 62 captured cases**, which pins the whole
Region 2 / 2.5 / error sub-tree onto one path.

## Seven of the thirteen assignment sites are DEAD in all 27 scenarios

Read out of `coverage/line_coverage.json` against the clean source, per site:

| clean line | writes | hits, all 27 scenarios |
|---|---|---|
| 220 / 222 / 224 | init: `PC_State_Enabled`, region-3 `ConstPwr`, `ConstTrq` | **0** |
| 227 / 228 | init: `Region_2` + `PC_State_Disabled` | 24 |
| 235 | op: `PC_State_Enabled` | 407,952 |
| 237 | op: `PC_State_Disabled` | **0** |
| 243 | op: `Region_3_ConstPwr` | 119,312 |
| 245 | op: `Region_3_ConstTrq` | **0** |
| 250 | op: `Region_2_5` | **0** |
| 252 | op: `Region_3_FBP` | **0** |
| 256 | op: `Region_2` | 15,986 — **scenario 12 only** |
| 259 | op: `Region_1_5` | 272,654 |
| 261 | op: `VS_State_Error` | **0** |

Every scenario's first call takes the init branch's *Region 2* leaf, so the
region-3 initialisation sub-tree — three of the thirteen — is entered by no
call in the whole campaign. `PC_ControlMode` is 1 in all 14 `Examples/*.IN`,
`VS_ConstPower` is the constant-power value wherever region 3 is reached, and
`VS_FBP` never takes the variable-pitch value where the torque test can see it.
**The differential harness is the only layer that reaches those seven**, which
is what the mutation score below is measuring.

## The gate sees this unit, and the first red test measured the perturbation

`gate/StateMachine.redtest.json` — `VS_State_Region_1_5` written as
`VS_State_Region_2` moves **36,577 of 5,252,000**, revert returns 0,
`scenarios_failed: []`, `perturbation_broke_scenarios: []`. Every moved channel
is in **scenario 12** (`bld_pitch` 115/16000, `gen_power` 15998/16000,
`gen_speed` 15997/16000, `gen_torque` 4237/16000).

That is a narrow red, and reading it as the unit's footprint would have been
wrong — the five scenarios that write `Region_1_5` hundreds of thousands of
times moved **nothing** under it. The whole-unit no-op is the habit unit #4
recorded, and it is what sizes the footprint:
`gate.whole-unit-no-op-MOVES.json` moves **1,526,538 of 5,252,000 across 22 of
27 scenarios and 135 channels**. So the gate is not blind to this unit at all;
the first perturbation simply chose a state change most scenarios' torque law
does not resolve. Both artifacts are kept, and the pair is the point: **a red
test proves visibility, it does not measure it.** Unit #12's check applied —
neither 36,577 nor 1,526,538 matches any other committed redtest artifact.

## The mutation score went 0.769 → 1.000, and the corpus is why

The first run left **9 of 39 mutants alive** on a 2028-case green. Not one was
a transcription defect: `branch_reachability.2028-case-corpus.txt` — the
shipped translation instrumented with a counter per leaf, run over the corpus —
shows **two whole sub-trees entered by zero cases**, and all nine survivors
live inside them.

Two gaps in `harness/generate.py`, both closed by ADDITION and both with a
precise detector:

1. **A predicate against a named PARAMETER was invisible.**
   `predicate_knobs_from` matched `NAME <op> NUMERIC LITERAL` only, so
   `IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr)` and
   `IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch)` produced no knob — the
   constants are `INTEGER(IntKi), PARAMETER` in `Constants.f90`. A reference
   that NAMES its magic numbers is being clearer than one that inlines them,
   and it was the clearer spelling the corpus could not read.
   `named_constants_from` harvests them from the reference's own directory; a
   name is admitted only where the reference's own predicate tests it, so the
   breadth of the scan costs nothing.

2. **A predicate between TWO VARIED QUANTITIES has no crossing value in any
   ladder.** `IF (LocalVar%PitCom(1) >= LocalVar%VS_Rgn3Pitch)` was FALSE in
   all 2028 cases, because `PitCom` is filled by `_fill_array`'s ascending ramp
   and `VS_Rgn3Pitch` sits at the scalar default — and **whichever base value
   is larger is larger in every case this generator has ever produced.** The
   crossing point is not a magnitude and not a literal: it is the other side's
   value in that same case, which only a rule that reads the predicate can
   know. `relational_pairs_from` finds all five of them here, and the new R7b
   block sets one side FROM the other at equality, under every knob
   combination.

   The first version of R7b was not enough and the reason generalises: a
   two-sided predicate can be GATED by another one. `GenArTq >= VS_MaxOMTq*1.01`
   sits inside the ELSE of `BlPitchCMeas >= VS_Rgn3Pitch`, which the ordinary
   draw made true in 2451 of 2890 cases, so `VS_State_Region_2_5` was still
   written by no case at all. R7b now runs each pair with **every other pair
   pinned just below its crossing point, and again just above** — the same
   conjunction argument R7 already makes for knobs, one predicate shape over.

2028 → 2890 → **3610 cases; all 16 branch leaves reached; 39 of 39 killed.**
The three `branch_reachability.*.txt` files are that progression, measured.

## Files

| file | what it is |
|---|---|
| `statemachine.final.cpp` | the shipped translation |
| `kernel.verification.run.txt` | `vit verify`'s own output, including its `NON_DISCRIMINATING` verdict |
| `kernel.verification-PASSES-62of62.run.txt` | the kernel's own stdout: 14,260 rows, all IDENTICAL |
| `kernel.verification-PASSES-62of62.verify_fields.csv` | the field log, which does not survive `reset_to_clean.sh` |
| `kernel.no-op-stub.run.txt` / `statemachine.no-op-stub.cpp` | 60 of 62 PASS — the mirror |
| `kernel.right-constant-stub.run.txt` / `.cpp` | 61 of 62 PASS — the lookup table |
| `kernel.wrong-constant-stub.run.txt` / `.cpp` | 0 of 62 — the comparison is alive |
| `kernel_field_rows.txt` / `.py` | verdict line vs row statuses, for all four runs |
| `statemachine.branch_probe.cpp` | the translation with a counter per leaf |
| `branch_reachability.2028/2890/3610-case-corpus.txt` | which leaves each corpus reached |
| `gate.whole-unit-no-op-MOVES.json` | 1,526,538 of 5,252,000 — the unit's gate footprint |

`vit verify` reports **`NON_DISCRIMINATING`** for this unit and is right to: it
constructs its automatic red test by perturbing a by-value floating-point
argument, and every input here arrives inside a derived type. That verdict is
kept in the evidence rather than argued away, and the three stubs above are the
red test run by hand in its place.
