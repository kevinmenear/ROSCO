# Unit #48 — `AeroDynTorque`

`rosco/controller/src/Functions.f90:411-450` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and one honest
failing number: the mutation score is **0.9062** against a threshold of 1.000.

**SECOND DISPATCH (2026-08-17).** Every number below that names 1131 cases or
0.8750 is the FIRST dispatch's and is left as measured; the sections it wrote are
marked where they are superseded rather than rewritten. What changed: each
survivor was put to the question *could a GREEN case kill this at all*, one was
`(b)` and was killed by fixing the inputs, and two are `(c)` — unkillable by this
harness on any input, which is why P12 cannot reach 1.000 here.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/AeroDynTorque.json`) | **1373 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, the `interp2d_c` bridge KEPT so both sides run one interp2d — this unit's primary evidence | the unit as a no-op: **1369 of 1373**, same corpus count |
| mutation (`mutation/AeroDynTorque.json`) | **29 of 32 scoreable, 0.9062**, 3 declared, 0 no-compile, 10 operators, **3 survivors standing** | the score *is* the red test, 29 times |
| post-integration (`harness/AeroDynTorque.postintegration.json`) | 1373 checked, 0 failed | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **1330 of 1373**; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/AeroDynTorque.json`) | 5,252,000 values / 351 channels, 0 mismatched | `PI` in the 13th digit moves **145,146**, 21 of 351 channels, scenarios 1, 7, 8, 12, 16, 25; revert verified at 0 |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for `interp2d` (unit #45) — this
unit's callee — and for units #46 and #47.

## The finding: the staging-refusal convention is not compositional

**THE FIRST TAKE WAS RED AT 14 OF 1387 AND IS COMMITTED**
(`harness.first_take.staging_composition.json`, and
`harness.first_take.all_14_cases.json` with the emitted test's own 16-diff cap
raised so every failing index is recorded).

All fourteen are `ErrVar.ErrMsg` / `n_ErrMsg`, contiguous at cases 1140..1153,
and the window is exactly `[L_callee, L_callee + L_prefix)` = **[16, 30)** in the
staging capacity R13 sweeps:

```
reference chain   Fortran -> Fortran -> Fortran, ONE capacity gate (the
                  generated bridge), on the final 30 bytes
translation chain C++ -> interp2d_c bridge -> Fortran, TWO gates: the bridge's
                  write-back on 16, then assign_errmsg on 30

cap  <  16      both refuse everything            got  7   ref  7   PASS
16 <= cap < 30  the bridge writes, assign refuses got 16   ref  7   FAIL
cap >= 30       both write                        got 30   ref 30   PASS
```

`interp1d` (unit #23) established "refuse rather than truncate"; `interp2d`
(unit #45) inherited it and never composed, because its own prefix is reached
only on the bilinear path and the bilinear path is the one path on which it does
not call `interp1d`. **`AeroDynTorque` is the first unit in this campaign in
which two staged CHARACTER assignments land in one buffer**, and R13's own
header states the model it was built on: "the translation refusing an assignment
that does not fit and the generated Fortran bridge refusing *the same one*."

Full arithmetic, both probes and the general statement:
`harness.staging_composition.txt`. Raised in `DECISIONS.md` as a proposed method
amendment, with the shape of the fix.

**Resolved by the first dispatch with `--disable R13_staging_capacity`, and the
price measured for all four survivors rather than argued**
(`mutation.survivors_on_full_corpus.txt`):

```
BASELINE   14 failed   cases 1140..1153
88466711   15 failed   cases 1140..1153 + 1154   <- killed by R13
11c1e326   14 failed   identical set             <- not
06f7d2c8   14 failed   identical set             <- not
26021804   14 failed   identical set             <- not
```

One mutant, one case. `88466711` is left standing as an honest survivor with
that case named — **five other units declared this same site unreachable and
this one does not**, because interp2d's R13 block never reached `assign_errmsg`
and this unit's would.

## One tool defect, fixed rather than routed around (X2)

`--disable <rule>` made the artifact say the rule had **no site**:

```
N/A  R13_staging_capacity  no deferred-length CHARACTER output -- no parameter
                           whose LENGTH the callee chooses
```

— because every rule's `if <sites> and "<rule>" not in off:` leaves its detail at
the string it was *initialised* to, and that string is the "found nothing to do"
sentence. A reader concludes the rule was inapplicable, when it was switched off
and its cases are missing from the corpus every later number is taken over.
Fixed in `translation-loop@b875e83`, once, after the table is built. Three
controls: the corpus hashes identically either side of the change
(`0893b9eb…8972`, so it is additive — P5), the ablated row now says ABLATED
without quoting the uncomputed detail as a counterfactual, and a misspelt
`--disable` name raises `ValueError` instead of silently ablating nothing.

## A gate blind spot, found by reading the red test's own scenario list

This unit has **two** call sites. The I&I one (clean
`ControllerBlocks.f90:389`, `WE_Mode == 1 .AND. WE_Op > 0`) is reached by
**scenario 17 and by nothing else**, 15,062 hits — and scenario 17 did not move
under the red test. Two probes:

```
gate.scenario17-probe.json    PI -> 3.2 (1.9% of the rotor area)   0 of 208,000
gate.scenario17-probe2.json   the return value forced to >= 1.0e5  0 of 208,000
```

Scenario 17 is alive: 16,000 distinct `gen_speed` values in its baseline. The
cause is `Examples/DISCON.IN:123`, `WE_Gamma = 0.0`, and the arm multiplies
`Tau_r` by it. **A call site with fifteen thousand hits whose returned value is
annihilated by a gain of exactly zero.** The hit count says nothing about it.

## What each file is

| file | what it records |
|---|---|
| `aerodyntorque.final.cpp` | the translation as scored |
| `harness.staging_composition.txt` | the 14-case first take: the arithmetic of the window, both probes, and the general statement |
| `harness.first_take.staging_composition.json` | that first take, kept: 1387 checked, 14 failed |
| `harness.first_take.all_14_cases.json` | the same run with the test's 16-diff cap raised, so all 14 indices are recorded |
| `run_harness_redtest.sh`, `aerodyntorque.noop-harness-stub.cpp` | the no-op red test and its runner (the stub keeps a guarded `interp2d_c(` so the callee bridge is still emitted — unit #45's rule) |
| `mutation.census.txt` | all four survivors, the three declarations, the measurements — with the second dispatch's dispositions at the top and two readings marked SUPERSEDED at their own headings |
| `aerodyntorque.green-kill-probe.cpp`, `mutation.green_kill_probe.txt` | **the decisive probe**: for each survivor, does ANY input exist with `ORIGINAL == REFERENCE` and `MUTANT != REFERENCE`. 203,700 configurations, both chains executed, negative control 0. 88466711 → 2100, 11c1e326 → 18, 06f7d2c8 → 0, 26021804 → 0 |
| `aerodyntorque.trim-composition-probe.cpp` | the probe's FIRST model, which was wrong and is kept: it tied the callee's message length to the entry's, and interp2d/interp1d REPLACE ErrMsg rather than only prefixing it |
| `mutant.<id>.cpp` | each survivor as `CppMutant.source`, so what was measured is the mutant that was scored |
| `run_mutation_part.sh` | one guarded, operator-filtered part |
| `run_r13_price.sh`, `harness.r13-boundary-mutant.FULL-corpus.json` | the first price measurement, for one survivor |
| `run_survivors_on_full_corpus.sh`, `mutation.survivors_on_full_corpus.txt` | the price measured for **all four**, by failing case SET rather than by count |
| `aerodyntorque.trim-census-probe.cpp` | the shipped translation with six counters; `TRIMCENSUS calls=1097 n_le_0=0 n_eq_1=0 n_min=9 n_max=21 with_trailing_blank=0` |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, 1330 of 1373 (1097 of 1131 in the first dispatch) |
| `gate.redtests.txt` | the one gate red test, its per-scenario channel list, and the scenario-17 blind spot |
| `gate.scenario17-probe.json`, `gate.scenario17-probe2.json` | the two probes that named it |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../mutation/AeroDynTorque.equivalences.{json,md}` | the three declarations and their proofs |

## The findings worth carrying

1. **A staging-refusal convention that is correct for one assignment is not
   correct for two.** The reference chain gates once at the outermost bridge;
   the translation chain gates at every nested boundary. First unit to compose.
2. **`--disable` must say it ablated.** A rule that did not run cannot report
   whether it had a site, and the initialised detail string says it had none.
3. **A hit count does not say a call site is observable.** 15,062 hits, a gain of
   0.0, and two probes moving 0 of 208,000.
4. **Measure the price of an ablation on every survivor, not on the one that
   motivated it.** That measurement is what made the second dispatch's fix
   possible: it named `cap == 30` as the one case R13 would have killed, so the
   ablation could be replaced by a stated hole of the fourteen cases that have no
   oracle — 242 capacities MORE than the ablation carried — and `88466711` died.
5. **"The corpus does not contain the killing input" and "no corpus can" are
   different findings, and a census cannot tell them apart.** Ask instead whether
   `ORIGINAL == REFERENCE` and `MUTANT != REFERENCE` can hold together. Two of
   these four survivors are separated from the original at 1,680 and 13,344
   configurations and **every single one is already red**.
6. **Execute an equivalence argument before believing it.** The first model of
   the green-kill probe proved three mutants behaviour-preserving from a clean
   inequality and was false. Running it is what caught it.
7. **The reference's own parentheses are load-bearing and the sweep says by how
   much.** `PI*(R*R)` → `(PI*R)*R` kills 52 of 1131 and
   `0.5*(rho*area)` → `(0.5*rho)*area` kills **1** of 1131 (first dispatch's
   corpus). One case out of eleven hundred is the entire margin between a
   transcription and an algebraic rearrangement.
