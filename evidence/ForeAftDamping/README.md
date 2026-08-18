# Unit #52 — `ForeAftDamping`

`rosco/controller/src/Controllers.f90:586-606` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and the mutation
score is **1.000** against a threshold of 1.000.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ForeAftDamping.json`) | **7567 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, the one callee bridge kept so each side runs one `PIController` — this unit's primary evidence | the unit as a no-op: **7567 of 7567**, the whole corpus, the same count, and the count the stub predicted in its own header |
| mutation (`mutation/ForeAftDamping.json`) | **8 of 8 scoreable, 1.0000**, 1 declared, 0 no-compile, 4 operators of 12 offered, **no survivor, `declared_but_killed` empty** | the score *is* the red test, 8 times |
| post-integration (`harness/ForeAftDamping.postintegration.json`) | 7567 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **7525 of 7567**; the 42 that pass are named exactly; reverted, rebuilt, green re-taken at 0, revert checked both ways |
| gate, 27 scenarios (`gate/ForeAftDamping.json`) | 5,252,000 values / 351 channels, 0 mismatched | **THREE, AND TWO OF THEM MOVE NOTHING**: the stored value + 0.01 rad moves **311,723** across the three scenarios that call it; the same statement × 2.0 moves **0**; the loop bound `<=` → `<` moves **0** |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45 through #51, and it is
the right one here for a reason of this unit's own: its `PIController` callee
carries per-instance state indexed by an `objInst` counter it post-increments,
and unit #44 measured that a KGen kernel cannot replay SAVE-like state.

## What this unit is

Two statements and no branch:

```fortran
LocalVar%FA_AccHPFI = PIController(LocalVar%FA_AccHPF, 0.0_DbKi, CntrPar%FA_KI, &
    -CntrPar%FA_IntSat, CntrPar%FA_IntSat, LocalVar%DT, 0.0_DbKi, &
    LocalVar%piP, LocalVar%restart, objInst%instPI)

DO K = 1,LocalVar%NumBl
    LocalVar%FA_PitCom(K) = LocalVar%FA_AccHPFI
END DO
```

`kp = 0.0` with `ki = FA_KI` makes the call a pure integrator, turning the
high-pass-filtered fore-aft ACCELERATION into a velocity — the field comments
say so (`ROSCO_Types.f90:339-340`). The caller adds `FA_PitCom(K)` to the pitch
command at `Controllers.f90:277`.

Two things are worth knowing before reading the translation:

1. **`-CntrPar%FA_IntSat` is a UNARY negation and is transcribed as one**, which
   looks like the opposite of what unit #51 did one procedure over.
   `floatingfeedback.cpp` writes `(0.0 - FA_vel)` because ITS reference writes
   `(0.0_DbKi - FA_vel)`. The two spellings differ at a zero (`0.0 - (+0.0)` is
   `+0.0`, `-(+0.0)` is `-0.0`) and this harness compares bit patterns. Both
   units transcribe the shape; they look opposite because the two references
   are. `FA_IntSat` is a configured real and 0.0 is inside the harness's draw
   for it, so the distinction is live rather than theoretical here too.
2. **`LocalVar%FA_PitCom` is `REAL(DbKi) :: FA_PitCom(3)`** — a fixed-size field,
   not an ALLOCATABLE — so it crosses inside the view struct with no synthesised
   extent, and the loop bound `NumBl` is unguarded against 3. A NumBl above 3
   writes past the field in the REFERENCE, which is why `harness/ranges.toml`
   states the domain.

## The finding: a corpus lever priced, rather than argued

The first range pin was `LocalVar_NumBl = { lo = 0, hi = 3 }` — the spelling
three earlier units carry. A census of the corpus it produced
(`numbl_census.txt`, take 1) says what that costs:

```
2539 cases
NumBl   0: 14    1: 2505    2: 12    3: 8
```

98.7% at `NumBl == 1`, because a bounds pin drops the parameter out of R6's
integer ladder and leaves the base draw at one value (unit #49's finding). At
`NumBl == 1` the loop runs once with `K == 1`, where `K - 1` and `1 - K` are
both 0 — so the `swap_operands` mutant on this unit's **only** index arithmetic
was killed on **20 cases of 2539**, and 20 is exactly the 12 + 8 with
`NumBl >= 2`. The value ROSCO itself runs, `NumBl == 3` in all three gate
scenarios, had 8.

`{ values = [3, 0, 1, 2] }` makes NumBl a second FLAG, so R2 and R6 re-run their
ladders under every declared value:

```
7567 cases
NumBl   0: 1890    1: 1894    2: 1893    3: 1890
```

Every kill count in the re-take then matches the census **to the case**:

```
compare_op    '<=' -> '<'         5677 = NumBl >= 1
arith_op      'K - 1' -> 'K + 1'  5677 = NumBl >= 1
const_tweak   '1' -> '2' (x2)     5677 = NumBl >= 1
swap_operands 'K - 1' -> '1 - K'  3783 = NumBl >= 2      (was 20)
const_tweak   I0 '0.0' -> '1.0'   3776 = restart == 1
```

Unit #50's `CntrPar_Flp_Mode = { values = [2, 0, 1, 3] }` is the same lever
pulled for the same reason. The difference is that #50 stated it in ADVANCE and
therefore could not price it — there was no dispatch on the other side of it.
Here both sides exist: `mutation/ForeAftDamping.clean.first_take.json` (2539
cases) and `.second_take.json` (7567 cases) are the same 9 mutants scored either
side, with the translation byte-identical across both.

## One survivor, and it is (c)

| site | mutants | why unkillable |
|---|---|---|
| `foreaftdamping.cpp:115`, the `1` of `LocalVar->restart ? 1 : 0` | 1 (`const_tweak '1' -> '2'`) | the callee bridge converts with `(reset /= 0)` and the integrated `picontroller.cpp:30` with `(reset != 0)`, so `? 2 : 0` and `? 1 : 0` are one argument on both trees |

**(c)-class** in unit #48's vocabulary — behaviour-preserving everywhere, not
merely unreachable. The same declaration units #50 (`flapcontrol.cpp:217`) and
#51 (`floatingfeedback.cpp:160` and `:163`) made for the same token.

**EXECUTED, NOT ARGUED** (`equivalence_probe.txt`), with three numbers:

```
restart == 1  -- the `? 1` branch is TAKEN     3776 of 7567   <- positive control
'0' -> '1' on the SAME conditional, line 115   KILLED at 3389 <- counter-control
I0 '0.0' -> '1.0', line 114                    KILLED at 3776 <- arithmetic control
```

The third is the one worth keeping: `PIController` returns its `I0` on precisely
the reset path this token selects, so that kill count agreeing with the census's
`restart == 1` count to the case is a control on the census rather than another
argument for it.

## The gate is green and it constrains nothing this unit computes

**ALL 21 `Examples/DISCON*.IN` SET `FA_KI = 0.00000` AND `FA_IntSat = 0.00000`.**
`PIController` ends `saturate(PTerm + ITerm, minValue, maxValue)` and this call
site passes `minValue = -FA_IntSat`, `maxValue = +FA_IntSat`. With both at zero
the result is clamped to zero for every input — and `ki` is zero too, so the
integrator never accumulates in the first place. This unit's answer is therefore
**exactly 0.0 on every one of the 63,997 calls the 27 scenarios make**, and what
the gate observes is `PC_PitComT + 0.0`.

That is why the same statement gives opposite answers under two perturbations:

```
the stored value + 0.01    311,723 of 5,252,000    scenarios 3, 7, 27
the stored value * 2.0             0               <- the SAME statement
the loop bound <= -> <             0
```

Unit #48's separating probe, run deliberately rather than met by accident: an
additive perturbation moves whatever the value is, a multiplicative one cannot
move an exact zero. So the zeros are ANNIHILATION, not blindness — the gate does
reach the statement, and the third blade is alive (`bld_pitch_3` moved in all
three scenarios under the additive run).

**Fifth instance in six units of an exact zero annihilating a gate
perturbation** — #47's `AWC_amp`, #48's `WE_Gamma`, #49's unwritten filter
state, #50's 1-DOF `rootMOOPF` — and the widest by far: those were one scenario
or one arm, this is every scenario the campaign has. Any translation that
returns zero passes this gate, including one that never calls `PIController`.
The differential harness and the mutation sweep are the instruments that
constrain the arithmetic.

## The post-integration red test's pass set, named

The copy-back red test predicted the whole corpus in its own header and was
wrong by 42. `nomove_census.txt` measures the pass set instead of arguing about
it:

```
whole LocalVariables unchanged      42 of 7567
  restart == 1                      42 of 42
  NumBl   == 0                      42 of 42
  incoming FA_AccHPFI == 0.0        42 of 42
```

Each conjunct kills one of this unit's three LocalVariables effects: `restart`
makes `PIController` return its `I0 = 0.0` and write that into a
zero-initialised `piP`; `NumBl == 0` deletes the loop; `FA_AccHPFI == 0.0` makes
the surviving assignment write 0.0 over 0.0. **The pass condition is that the
unit changed nothing in the type, not that it wrote nothing** — which is the
correction the prediction needed. `objInst%instPI` is not in it and could not
be: `objInst` crosses by `C_LOC` rather than through a view struct.

## `--reverse-copy` was decided by reading the emitted wrapper

The first `vit integrate --apply` was run without the flag and the wrapper it
emitted carried **no copy-back at all** — populate, call, return — which would
have left both of this unit's LocalVariables writes dying inside the view
struct, with the harness and the gate green on it (unit #23's shape). Reverted
and re-applied with the flag; no artifact was ever taken against the wrong
wrapper. The re-applied wrapper carries a `vit_copy_scalars_to_controlparameters`
call as well, which is correct and inert: ROSCO declares `CntrPar` INTENT(INOUT)
here and this unit assigns nothing in it. `vit integrate` again rewrote
`vit.yaml` through a YAML round trip, as units #14, #49 and #51 record.

## What each file is

| file | what it records |
|---|---|
| `foreaftdamping.hstub-noop.cpp`, `run_harness_redtest.sh` | the no-op red test and its runner (the stub keeps one guarded, unreachable `picontroller_c` call so the bridge is still emitted — unit #45's rule; the runner asserts the translation is in HEAD before arming a trap that would delete it) |
| `harness.noop.json` | that red test: **7567 of 7567**, the whole corpus, as the stub predicted |
| `numbl_census.txt` | both censuses — the concentrated corpus the bounds pin produced and the near-uniform one the `values` list produces — and the arithmetic that ties each mutant's kill count to a case count |
| `equivalence_probe.txt` | the executed argument behind the one declaration, with a positive, a counter- and an arithmetic control |
| `run_mutation_part.sh` | the guarded sweep runner (`all` runs the whole population in one part; the operator split exists for 192-mutant units, not as a rule) |
| `../../mutation/ForeAftDamping.clean.first_take.json` | the sweep over the 2539-case corpus the bounds pin produced, `equivalent_declared: 0`, 8 of 9 at 0.8889 |
| `../../mutation/ForeAftDamping.clean.second_take.json` | the same 9 mutants over the 7567-case corpus, still undeclared, still 8 of 9 — the score did not move, the kill COUNTS did |
| `../../mutation/ForeAftDamping.clean.final.json` | the re-take with the declaration, 8 of 8 at 1.0000, byte-identical to `mutation/ForeAftDamping.json` (sha256 `5fd44677e48ddfe2…`) |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, **7525 of 7567** |
| `nomove_census.txt` | the 42 that survive it, characterised exactly |
| `gate.redtests.txt` | the three gate perturbations, the DISCON evidence for the zero, and the additive/multiplicative pair that separates annihilation from blindness |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../harness/ranges.toml` | two pins: `LocalVar_NumBl` (with both takes and the price of the first) and `objInst_instPI` for the thirteenth time |
| `../../mutation/ForeAftDamping.equivalences.json` | the one declaration and its proof |

## The findings worth carrying

1. **A bounds pin's base draw is a CORPUS SHAPE and it is measurable in one
   probe.** `{ lo = 0, hi = 3 }` put 98.7% of the cases at one value and left
   this unit's only index arithmetic resting on 20 of 2539. Three earlier units
   carry the same pin and nobody had looked.
2. **A `values` list whose members ARE the whole admissible interval costs
   nothing that the bounds pin was not already costing**, and buys the ladders
   re-run under every value. That is a cheap lever, and it is only cheap when the
   interval is small enough to enumerate.
3. **Run the additive/multiplicative pair on the SAME gate statement when a
   perturbation moves zero.** Unit #48 discovered the shape by accident; running
   it on purpose costs one gate run and turns "the gate is blind here" into "the
   value is exactly zero here", which are different facts about the campaign.
4. **A copy-back red test's pass set is "the unit changed nothing in the type",
   not "the unit wrote nothing".** The distinction is worth a probe whenever the
   predicted count and the observed one differ.
