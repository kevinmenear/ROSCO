# STATUS — rosco-r2

**This is the live state file. Read it first, then `RUNBOOK.md`.**
`DECISIONS.md` is the append-only record of *why*; this file is *where things
stand*. One copy of every count — do not duplicate them anywhere else.

**As of 2026-08-15: unit #42 `Startup` is `integrated` and CLOSED** — all five
layers exist, all five ran, all five are red-tested, and the mutation score is
**1.000** on 102 behavioural mutants with 4 declared. It is the campaign's first
unit whose EXTRACTION WINDOW WAS FOUR RANGES AIMED AT STATE-MACHINE EDGES rather
than start/middle/end, and the first whose kernel red test names a single case
index predicted in advance.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 83 cases (`evidence/Startup/kernel.verify_fields.csv`) | 83/83, all 19,671 field rows `IDENTICAL` | **`vit verify` DECLINED to build one** and printed `NON_DISCRIMINATING`, third unit running. Three hand stubs instead: no-op **0 of 83 pass**; the freewheel low-speed write deleted **83 of 83 PASS**; the startup-complete reset deleted **fails EXACTLY ONE case, `Startup.0.0.1801`** |
| differential harness (`harness/Startup.json`) | **7151 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **7103 of 7103**, on the corpus before the baseline states were added — the same count that green certified |
| mutation (`mutation/Startup.json`) | **102 of 102 behavioural, 1.000**, 4 declared, 0 no-compile, 6 operators of 12 offered | the score *is* the red test, 102 times |
| post-integration (`harness/Startup.postintegration.json`) | 7151 checked, 0 failed | the wrapper's scalar copy-back deleted: **7151 of 7151** |
| gate, 27 scenarios (`gate/Startup.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO: `PRC_R_Torque = 0.0 -> 1.0` moves **34,086**; the freewheel low-speed arm moves **0** |

**THE WINDOW WAS ARITHMETIC AND THE STUB CONFIRMED IT TO THE CASE INDEX.**
`DISCON.F90:112` sits behind `IF (CntrPar%SU_Mode > 0)` and carries hits under
scenario 9 alone — 11,999 invocations, 0 in the other 26, the same shape as unit
#41. But this unit's whole life is over by invocation **1,801**: the three stage
transitions are at 201, 1001 and 1801, computed from scenario 9's own patch dict
(dt = 0.025, `SU_FW_MinDuration` = 5, `SU_LoadRampDuration` = 10 10,
`SU_LoadHoldDuration` = 10 10) and each cross-checked against a coverage hit
count before the window was written. A start/middle/end window would have put two
of three ranges in `SU_Stage == 0`, where the body writes nothing. Four ranges,
83 cases, and the capture agrees to the invocation: case 200 leaves `SU_Stage` at
1 and 201 at 2, 1000 at 2 and 1001 at 3, 1800 at 3 and 1801 at 0. Deleting the
`SU_Stage = 0` reset then fails exactly `Startup.0.0.1801` and nothing else.

**SIX OF THE FIRST SWEEP'S FOURTEEN SURVIVORS WERE ONE FACT ABOUT `sigma`.**
Every stage-2 case the generator produced had `Time < SU_LoadStageStartTime`, so
the call took its `y0` early return and neither the ramp-end expression, nor its
subscript, nor `y1` could reach an answer. The same sweep killed `drop_factor` on
`y0` on 624 cases, which is what says the corpus reached exactly one arm of one
callee rather than having six independent holes.

```
                                    cases   mutants killed / 106
  no baseline                        7103   92          0.868
  + 3 states (threshold, cubic,      7151   102         1.000 with 4 declared
    past-ramp)
```

**A THRESHOLD WHOSE LEFT SIDE THIS UNIT COMPUTES, REACHED BY UNIT #41's ROUTE.**
`SU_RotSpeedF < 0.95 * SU_RotorSpeedThresh` compares a computed value against a
given one, so R6's relational-pair rule — which needs both sides to be inputs —
cannot put it at equality. `LPFilter`'s initialisation arm returns its input
BIT-EXACTLY at `CornerFreq = 0`, which turns the computed side back into an
input. What had to be checked rather than assumed: `iStatus = 0` forces
`SU_Stage = -1` two statements earlier and the statement between them promotes
-1 to 1 whenever `Time > SU_StartTime`, so the arm survives the setting that
reaches the answer.

**ONE STATEMENT IS OUTSIDE EVERY SIMULATION THIS CAMPAIGN RUNS, AND THREE
INSTRUMENTS SAY SO.** `ControllerBlocks.f90:587` carries 0 hits in all 27
scenarios; the gate red test on that arm moves **0 of 5,252,000**,
revert-verified; and the kernel stub with the write deleted passes **83 of 83**.
Scenario 9 starts the rotor at 4 rpm = 0.419 rad/s against a threshold of
0.95 * 0.3 = 0.285, so no window into scenario 9 could have reached it. The
differential harness is the one instrument that is not blind — `negate_cond` on
that condition is killed on 5618 of 7151 cases.

**A PERTURBATION THAT DOES NOT COMPILE IS NOT A PERTURBATION THE GATE CANNOT
SEE.** The second gate red test failed to build on its first attempt (`\&\&`
inside single quotes reaches the compiler as backslashes) and `scripts/gate.py`
printed `PERTURBED BUILD FAILED -- no red test was performed` rather than a zero.
On a red test whose EXPECTED result is zero movement, those two outcomes are
indistinguishable from the number alone, and the tool is what distinguishes them.

---

**As of 2026-08-15: unit #41 `Shutdown` is `integrated` and CLOSED** — all five
layers exist, all five ran, all five are red-tested, and the mutation score is
**1.000** on 97 behavioural mutants with 12 declared. It is the campaign's first
unit whose DIFFERENTIAL CORPUS NEEDED THIRTEEN BASELINE STATES, and the first
whose extraction scenario was not a choice: only 1 of the 27 scenarios calls it
at all.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases (`evidence/Shutdown/kernel.verify_fields.csv`) | 62/62, all 14,694 field rows `IDENTICAL` | **`vit verify` DECLINED to build one** and printed `NON_DISCRIMINATING`. Three hand stubs instead: no-op **0 of 61**; the vane arithmetic constant **62 of 62 PASS**; the SD_EnableTime arm deleted **moves exactly ONE case** |
| differential harness (`harness/Shutdown.json`) | **14708 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **14708 of 14708**, the same count the green certifies |
| mutation (`mutation/Shutdown.json`) | **97 of 97 behavioural, 1.000**, 12 declared, 0 no-compile, 8 operators of 12 offered | the score *is* the red test, 97 times |
| post-integration (`harness/Shutdown.postintegration.json`) | 14708 checked, 0 failed | the wrapper's scalar copy-back deleted: **14708 of 14708** |
| gate, 27 scenarios (`gate/Shutdown.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO: `SD_Trigger = 4 -> 0` moves **11,990**; the whole `SD_Method == 2` arm moves **0** |

**THE EXTRACTION SCENARIO WAS NOT A CHOICE, AND THE WINDOW WAS ARITHMETIC.**
`DISCON.F90:107` sits behind `IF (CntrPar%SD_Mode > 0)` and carries hits under
scenario 9 alone — 11,999 invocations, 0 in the other 26. Inside those, coverage
splits the call site 10,002 / 1,997 between the `SD_Stage == 0` arm and the stage
arm, so the state transition is at invocation 10,002 and the middle capture
window was aimed at 9995-10015 rather than at the midpoint. That is what the
kernel's sharpest number rests on: deleting the only trigger arm scenario 9 fires
moves **exactly one of 62 cases**, invocation 10002. On the other 34 cases where
`sd_trigger` reads 4, the 4 arrives already in the captured state and is copied
through — the arm is not computed there, it is inherited. A midpoint window would
have reported the same 62/62 green over a corpus in which that arm is dead.

**R6's ALL-PAIRS FALLBACK PUT ZERO CASES ON EVERY ARM THAT WRITES THIS UNIT'S TWO
PRINCIPAL OUTPUTS.** Unit #37's finding, met a second time and measured the same
way. Reaching `SD_MaxPitchRate(SD_Stage)` needs a TRIPLE — `SD_Method` in {1,2}
AND `SD_Stage /= 0` AND `SD_Stage <= SD_Stage_N` — and the coverage line reads
`283 combination(s) ... ALL PAIRS ONLY -- the full cross product is 186624, past
the 4096 bound`, which holds every uncrossed knob at its ladder's first value.
`SD_Method`'s first value is 0: neither method.

```
                                  cases   mutants killed / 109
  no baseline                     14253   80          0.734
  + 2 states (the two methods)    14323   91
  + 3 states (one per trigger)    14428   ...
  + 6 states (five on equalities) 14708   95
  + 2 states (threshold equality) 14708   97          1.000 with 12 declared
```

**TWO THRESHOLD BOUNDARIES LOOKED UNREACHABLE AND ARE NOT, AND THE ROUTE IS
WORTH KEEPING.** `SD_GenSpeedF > SD_MaxGenSpd` and `ABS(SD_NacVaneF) >
SD_MaxYawError` compare a value this unit COMPUTES against one it is given, so
R6's relational-pair rule — which needs both sides to be inputs — cannot put them
at equality. But `LPFilter`'s initialisation arm returns its input BIT-EXACTLY at
`CornerFreq = 0`: the coefficients become 2, -2, 0, 0 and the expression
collapses to `(1/2)*(2x)`, both operations exact. Setting the corner frequency to
zero with `iStatus = 0` turns the computed side back into an input, and both
mutants are killed rather than declared.

**A STATED RANGE DID NOT NARROW A PREDICATE KNOB, AND 31 CASES OF THE REFERENCE
READ BEFORE THE START OF AN ARRAY BECAUSE OF IT.** `LocalVar_SD_Stage = { lo = 0,
hi = 3 }` was stated because the reference's guard reads `SD_Stage .LE.
SD_Stage_N` and not `1 .LE. SD_Stage`. `predicate_knobs_from` derives its ladder
from the reference's predicates and never consults the signature, so the knob
supplied -1 anyway. The harness reported 31 failures naming `SD_MaxPitchRate` —
indistinguishable from a translation defect, and `vit_mutate.py` refuses to score
against a red baseline. Fixed where it lives (`translation-loop f92fb9f`, four
tests including the positive control that an UNSTATED bound must not narrow).

**THE FIRST CORPUS KILLED THE REFERENCE AT CASE 21, AND THE C++ SIDE HAD ALREADY
CORRUPTED IT BEFORE THE REFERENCE RAN.** `Error allocating 11703869192 bytes`
from the generated bridge, with `harness produced no JSON` — which is also what a
printing reference produces. Two `fprintf`s and one `WRITE(0,*)` named it in one
9-second run: the C++ read `n_PF_TimeStuck = 3` out of the case file and the
Fortran bridge was handed 1462983649, because between the two the only thing that
ran was the C++ translation, and `LPFilter` had written past `FP`'s six
`DIMENSION(1024)` arrays and out of `LocalVar_a` into the neighbouring stack
object. Fifth instance of one class; `objInst_instLPF = { lo = 1, hi = 1000 }`.
`evidence/Shutdown/harness.case21_probe.txt`.

**A MUTATION SWEEP WAS KILLED BY THE 600-SECOND TOOL CEILING AND LEFT A MUTANT
LIVE IN THE TRANSLATION.** `scripts/run_if_time_remains.sh` guards the DISPATCH
deadline and knows nothing about the per-command ceiling; a 69-mutant part needed
~800s and the clock said 8,400s remained, so it started. `mutate_guarded.sh`
caught it — refused to clear its marker, printed the intended hash — and the
`docker exec` had ORPHANED in the container and was still running. Killed,
restored, re-split into four parts of under 460s each.

---

**As of 2026-08-15: unit #40 `SetpointSmoother` is `integrated` and CLOSED** —
all five layers exist, all five ran, all five are red-tested, and the mutation
score is **1.000** on 16 behavioural mutants with 2 declared equivalent. It is
the campaign's first unit whose GENERATED CORPUS ANNOUNCED COVERAGE IT DID NOT
DELIVER, and the instrument had to be widened before any mutation number meant
anything.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases (`evidence/SetpointSmoother/kernel.verify_fields.csv`) | 62/62, all 14,508 field rows `IDENTICAL` | **`vit verify` DECLINED to build one** and printed `NON_DISCRIMINATING`. Six hand stubs instead: **0 · 62 · 0 · 23 · 62 · 0** |
| differential harness (`harness/SetpointSmoother.json`) | **13550 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **13144 of 13550**, naming `LocalVar.SS_DelOmegaF` |
| mutation (`mutation/SetpointSmoother.json`) | **16 of 16 behavioural, 1.000**, 2 declared equivalent, 0 no-compile, 6 operators of 12 offered | the score *is* the red test, 16 times |
| post-integration (`harness/SetpointSmoother.postintegration.json`) | 13550 checked, 0 failed | the `--reverse-copy` line deleted and rebuilt: **13144 of 13550** — the no-op's count to the value |
| gate, 27 scenarios (`gate/SetpointSmoother.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO: the `PC_RefSpd` scaling offset by 1e-6 moves **617,546**; `R_Total` dropped to 1.0 moves **56,128** |

**THE GENERATOR PRINTED `PREDICATE KNOB: CntrPar_SS_Mode at [0.0, 1.0, 2.0]` AND
PUT FOUR CASES OF 5599 ON THE BRANCH THAT KNOB NAMES.** Every statement this
unit computes lives on the `SS_Mode == 1` arm. `SS_Mode` is an INTEGER with no
declared values, so it never entered `generate.py`'s `flags` list and
`flag_variants` never re-ran R6's 1456 ladder values under it. This is P9 at the
level of the *instrument* rather than the scenario: coverage announced, not
delivered.

An arm census made it a count rather than a suspicion, and the partition closes
to the value on both corpora:

```
                        5599 cases      13550 cases
    IF arm deleted           4             4516
    ELSE arm deleted      5351             8628
                        ------           ------
    the no-op             5355            13144
```

Closed by ADDITION — `CntrPar_SS_Mode = { values = [0, 1, 2] }`, this file's
first `ranges.toml` entry that *widens*. It then killed the reference at case
8114 (`SS_Mode=1`, `instLPF=-999`, six `DIMENSION(1024)` arrays indexed 999
elements before their start), which forced a second entry —
`objInst_instLPF = { lo = 1, hi = 1000 }`, the bound three earlier units already
carry. **The 16 mutation kills are against 4516 IF-arm cases, not against 4.**

**ONE DEFECT SHAPE IS SEEN BY EXACTLY ONE OF THE FOUR INSTRUMENTS.** Rewriting
`(num/VS_RtPwr)*SS_PCGain` as `num*SS_PCGain/VS_RtPwr` — equal in real
arithmetic, differently rounded in IEEE, and the shape this campaign's own
guidance names — **passes the kernel 62 of 62**, and the mutation sweep is
*silent* on it: `assoc_reorder` is in this unit's `operators_offered` and
produced no mutant. The differential harness fails it on **35 of 13550**, at one
to two ULP, with the shipped translation as a control in the same window at
13550/0.

**A KILLED DISPATCH LEFT A MUTANT LIVE IN THE TRANSLATION, AND `git diff` WAS
THE ONLY THING THAT FOUND IT.** `run_kernel_stubs.sh` edits the shipped file in
place and restores on completion; it was killed inside its stub 3, leaving
`R_Total = 1.0;` in `translations/ControllerBlocks/setpointsmoother.cpp` across
the dispatch boundary. Every stub runner written since restores from
`git checkout` on an EXIT trap. Stub 3's verdict was finally taken this dispatch:
**0 of 62**.

**THE FORTRAN LEAVES ITS OWN RESULT UNASSIGNED, AND THE HARNESS FOUND IT BEFORE
ANY READER DID.** `ResController` assigns `ResController` in the ELSE arm only;
on `IF (reset)` it returns whatever the result slot holds, and
Controllers.f90:815 assigns that to `AWC_TiltYaw(Imode)` — the inverse Coleman
transform and every blade's pitch command. Upstream ROSCO's **fifth** recorded
defect. The first run of the corpus said so from the other side:

```
HARNESS FAIL: checked 3532  failed 1763  inadmissible 0
  case 13 vit_result: ref f2c28ae964943041 != got 0000000000000000
  case 17 vit_result: ref 777cb0d2ffff0000 != got 0000000000000000
  case 18 vit_result: ref 10cf0af41fb70000 != got 0000000000000000
```

Every kept mismatch names `vit_result` and nothing else, and the reference's
bytes change from case to case — `777cb0d2ffff0000` is a leftover pointer.

**`harness/ranges.toml` COULD NAME A STRUCT FIELD WITH NO ORACLE AND NOT A
FUNCTION RESULT, SO THE ONLY REMEDY AVAILABLE WAS THE WRONG ONE.**
`reset = { values = [0] }` would have thrown away the reset ARM — four writes
**no other instrument in this campaign can reach**. Fixed where it lives
(`translation-loop 04975cf`): `no_oracle` now names `vit_result` too, R4's own
coverage line stops claiming the return is compared, and the same 3532-case
corpus goes from `failed 1763` to `failed 0`. That transition is also the proof
that all 1763 were that one output; the mismatch list is truncated at sixteen.

The cost is small and it is measured rather than argued: the ELSE arm stores the
returned value into `res_OutputSignalLast1(inst)`, which is compared on every
case, so the arithmetic is not excluded — only the copy of it that leaves
through the return.

**THE SECOND GATE RED TEST MOVED 0 OF 5,252,000, AND THAT IS THE FINDING —
AGAIN.** Unit #38 recorded the same shape one unit ago.

```
the unit produces no output   saturate(...) -> 0.0        280,312 of 5,252,000
                                                          revert-verified 0
the reset arm                 one write 0.0 -> 12345.0          0 of 5,252,000
                                                          RED_TEST_FAIL, revert 0
```

`LocalVar%restart = (iStatus == 0)` is the only assignment to it in the
controller, and the AWC block sits behind `IF (LocalVar%Time .GT. StartTime)`,
false at `iStatus == 0`. So the arm is not merely unsampled by these 27
scenarios — nothing in them can enter it. The kernel says the same from the
value side (`restart` F in 62 of 62; the emptied-arm stub passes 62 of 62). The
differential harness reaches it in **1464 of 2925** cases and is the only thing
that does.

**THE FIRST MUTATION SWEEP SCORED 0.9275 AND THE KILL COUNTS ARE THE CENSUS.**
Bimodal, on the unpinned 3532-case corpus:

```
killed on    12 of 3532 cases   21 mutants   <- the ELSE arm's arithmetic
killed on    13/15/22/23        17
killed on  1763 of 3532 cases    8 mutants   <- the reset arm's four writes
killed on  1769 of 3532          6
killed on  3532 of 3532 cases    7 mutants   <- structural
```

1766 cases run the ELSE arm and about **twelve** of them let its arithmetic
reach a compared output. `saturate(x, lo, hi)` returns `hi` whenever
`lo >= hi`, R6 draws both bounds from one ±1e3 default, and its isolating stage
then pins every other real to 0.0 or 1e300 — which sets the two bounds EQUAL.
This is `[PIDController]`'s cause 2 met a second time, and the pins are the same
shape: the call site's own `PC_MinPit`/`PC_MaxPit`, widened. Three of the five
survivors died with them.

**ONE SURVIVOR WAS NOT AN EQUIVALENCE AND WAS CLOSED BY ADDITION (P5), AND IT
COST EXACTLY ONE CASE.** `2.0*(omega*omega)` → `(2.0*omega)*omega` is bit-equal
for every input in the NORMAL range — multiplying by two is exact, so
`2·RN(u)` and `RN(2u)` sit on grids one power of two apart. They part company
only where the grid stops being relative:

```
u = omega*omega = 0.4 D          2.0*RN(u) = 2.0*0.0 = 0.0
   (D = 4.94e-324)               RN(2u)    = RN(0.8 D) = D      <- one quantum
```

Reaching a compared output from there needs a SECOND quantity at its own
extreme, which is why no one-parameter ladder finds it: the quantum has to
survive `-8 +`, and `D * DT*DT` only clears half an ulp of 8 once `DT*DT` is
within a factor of two of DBL_MAX. One baseline state — `freq = 1.77e-163`,
`DT = 1e154`, every other term zero or one — makes `b1` come back **one ulp
apart** and the mutant dies on that single case. 0.971 → 0.986.

**THE LAST SURVIVOR IS DECLARED, AND ITS REASON IS THE SAME FACT AS THE
`no_oracle` ENTRY.** `double ResController_result = 0.0` → `= 1.0` is the
declaration's initialiser, read only on the reset path, where the reference has
no defined value at all. The initialiser exists so that the C++ is not undefined
BEHAVIOUR as well as an undefined VALUE; which constant it holds is arbitrary by
construction. Applied by hand to the shipped translation it passes **2925 of
2925** (`evidence/ResController/harness.eq-probe-c06837a3.json`), and it is the
only one of this file's five `0.0` literals that survives — the four reset-arm
writes are killed on 1464 of 2925 each.

**`b2 = b0` IS AN EQUIVALENCE IN THE WHOLE PROGRAM AND IT IS RECORDED, NOT
DECLARED.** Controllers.f90:1202 and :1204 both read `4+omega**2*DT**2` — same
expression, same operands, same order — so a stub reading `b2` off `b0` passes
62 of 62 and no instrument at any input could disagree. No mutant edited only
one of the two sites, so nothing needed declaring; it is written down so that
one which does is read as what it is.

**THE KERNEL IS NOT SATURATION-BLIND, WHICH IS WHAT SEPARATES THIS UNIT FROM
#33 AND #31.** The no-saturate stub fails **26 of 62**, so 36 captured cases
return the raw resonator output. `PIController`'s kernel returned `minValue` in
62 of 62 and was 45-of-62 blind to `error`; `PIDController`'s harness corpus
returned a bound in 2307 of 2307. Here `PC_MinPit` is 0.0 and `PC_MaxPit` is
1.57 and the AWC error signal drives the output across zero.

---

**As of 2026-08-14: unit #38 `PreFilterMeasuredSignals` is `integrated` and
CLOSED** — all five layers exist, all five ran, and the mutation score is
**1.000** on 114 behavioural mutants with 12 declared equivalent. It is the
campaign's first unit whose body is almost entirely CALLS — twenty of its
twenty-six executable statements are a filter call, and all six callees are
already integrated — and it took **two tool defects, eight range pins and a
corpus repair** before any instrument produced a number.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases (`evidence/PreFilterMeasuredSignals/kernel.verify_fields.csv`) | 62/62, 28,582 field rows `IDENTICAL` | five stubs: the no-op **0 of 62**, no-DebugVar **0**, no-notch-loop **0**, Moving-dropped **1** — and `reset` forced false passes **62 of 62** |
| differential harness (`harness/PreFilterMeasuredSignals.json`) | **9033 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **293 of 9033** |
| mutation (`mutation/PreFilterMeasuredSignals.json`) | **114 of 114 behavioural, 1.000**, 12 declared equivalent, 0 no-compile, 9 operators | the score *is* the red test, 114 times |
| post-integration (`harness/PreFilterMeasuredSignals.postintegration.json`) | 9033 checked, 0 failed | TWO: the LocalVar copy-back deleted **293 of 9033**; DebugVar/objInst transposed **293 of 9033** |
| gate, 27 scenarios (`gate/PreFilterMeasuredSignals.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO, and **one of them FAILED** — see below |

**THE GATE'S SECOND RED TEST MOVED 0 OF 5,252,000, AND THAT IS THE FINDING.**

```
the whole unit a no-op   IF (aviFAIL < 0) forced true   1,751,360 of 5,252,000
                                                        140 channels, revert 0
the Flp_Mode == 2 arm    disabled                                0 of 5,252,000
                                                        RED_TEST_FAIL, revert 0
```

Scenario 4 is the only one of the 27 that reaches that arm — 12,000 hits in
`coverage/line_coverage.json` — and it is a 1-DOF sim whose own docstring says
*"blade root moments are near-zero"*. Unit #35 measured the same thing from the
other end: its `PIIController` kernel found `-LocalVar%rootMOOPF(K)` identically
zero in every captured case. So that arm has a coverage number, a translation
and harness kill counts, and **no simulation evidence at all** — and it is where
this campaign's own fix to an upstream ROSCO indexing bug lives.

**EVERY ARM IS REACHED, AND THE COUNTS ARE A READING RATHER THAN A CLAIM.**
`probes/arm_census.cpp` is the shipped translation with one counter per arm; it
changes no compared output, so its run **fails 0 of 9033** and is a reading of
the corpus, not a perturbation of it.

```
calls                                  9033
  RETURNed on aviFAIL < 0              8740
  ran the body                          293
F_LPFType == 1                           42
F_LPFType == 2                           42   <- 0 hits in all 27 scenarios
F_LPFType matched NEITHER                209
gen-speed notch loop, iterations         232
iStatus == 0 .AND. Time == 0               2
tower-top notch loop, iterations         275   <- 0 iterations, all 27 scenarios
TD_Mode > 0                               93
blade loop, iterations                   292
  IPC_ControlMode>0 .OR. Flp_Mode==3     113
  Flp_Mode == 2                           90   <- scenario 4 only
    its inner notch loop, iterations     171
  the ELSE arm                            89
```

The three arms no bit-exact instrument sees are reached **42, 275 and 90**
times. 42 + 42 + 209 = 293 and 113 + 90 + 89 = 292 (one body-running case has
`NumBl <= 0`), so the partition closes.

**THE NO-OP MOVES ONLY 293 OF 9033 HARNESS CASES, SAID HERE RATHER THAN LEFT TO
BE FOUND.** A case the no-op passes is a case that never got past
`IF (ErrVar%aviFAIL < 0) RETURN`; **8740 of the 9033 are that**. R6's all-pairs
fallback again — the cross product of the ten quantities this unit's predicates
test is **552,960**, past `_KNOB_CASE_LIMIT` of 4096, so every knob not being
crossed sits at the first value of its ladder and `aviFAIL`'s first value is
**-1**. Closed *in part* by addition, and the addition is measured:

```
without harness/baseline.PreFilterMeasuredSignals.json   217 of 8955
with it                                                  293 of 9033
```

R11's 78 cases contributed 76 of the 293. The other 8740 did not move and the
number stands as it is.

**THE FIRST MUTATION SWEEP SCORED 0.8254 AND THE CAUSE WAS THIS UNIT'S OWN
BASELINE FILE.** Ten of its twenty-two survivors were `'ind - 1' -> '1 - ind'`
and `'n - 1' -> 'n + 1'` inside the two notch loops that only R11 reaches — and
the first baseline wrote `{"ramp": [1, 0]}` for both index arrays and 1 for both
notch counts, so `ind - 1` and `1 - ind` are **the same number** and a shifted
loop index reads an equal element. R11's own implementation comment records this
from unit #29 (49 of that unit's 89 survivors) and the file reproduced it. Two
counts and two ramps changed, the corpus stayed at 9033 cases, and the failing
artifact is kept at
`evidence/PreFilterMeasuredSignals/mutation.CONSTANT-INDEX-RAMP-9033.json`.

**C12 — `_mutation_merge.py` REFUSED THE MERGE, AND IT WAS RIGHT TO AND WRONG
ABOUT WHY.** `114 killed + 0 survived != 126 behavioural`: its identity check
did not know that `mutants` counts the declared equivalences while `killed` and
`survived` do not — although `denom = behavioural - eq` five lines below already
read it that way. Two statements of one identity, disagreeing, and the one that
ran first refused every merge with an equivalence in it. This unit is the
campaign's first SPLIT sweep declaring any equivalence, so the gap could not
have surfaced earlier: #36 declares six and ran in one part.

**THREE ARMS ARE OUTSIDE EVERY BIT-EXACT INSTRUMENT** (clean-source lines):
`F_LPFType == 2` has **0 hits in all 27 scenarios**; the tower-top notch `DO`
line carries **exactly the call count** in every scenario, so its body iterates
zero times; the `Flp_Mode == 2` arm is scenario 4 only. The kernel's 62 cases
confirm all three at the value level (`f_lpftype` 1, `f_twrtopnotch_n` 0,
`flp_mode` 0 in 62 of 62). The differential harness reaches them, and that is
why its R11 baseline states **two** configurations.

**THE STUB THAT PASSES 62 OF 62 IS AN EQUIVALENCE IN THE WHOLE PROGRAM, PROVED
BY TWO GREPS RATHER THAN A CASE COUNT.** All five filters guard initialisation
with `IF ((iStatus == 0) .OR. reset)`, and `ReadSetParameters.f90:123-126` is
the **only** assignment to `LocalVar%restart` in the controller —
`restart = (iStatus == 0)`. So `reset` is redundant at all nineteen of this
unit's call sites, forcing it false changes nothing at any window in any
scenario, and it predicts the gate is blind to that argument too. What is not
blind is the differential harness, which draws the two independently.

**THREE THINGS STOOD BETWEEN THIS UNIT AND ANY NUMBER**, each fixed where it
lives (X2) and each with its failing message recorded:

```
loop 3ac5b4a   an INDEX CAN BE AN ARRAY. F_GenSpdNotch_Ind carries role="index"
               AND dims, and all THREE sites that materialised an index wrote a
               scalar:  TypeError: 'int' object is not iterable
vit  fe22383   an unrestricted `USE M` puts M's names in scope, not every name:
               Error: Derived type 'filterparameters' ... used before defined
ranges.toml    SIGSEGV at case 17 of 9447, zero-byte artifact, `harness produced
               no JSON` -- five objInst counters subscripting DIMENSION(1024)
               inside the callees, and NumBl bounding a loop over DIMENSION(3)
```

The third is unit #34's `objInst%instLPF` judgement, needed a second time and
five times wider. The bisect is the measurement: those six pins alone take the
run from `exit 139, core dumped` to `checked 8955 failed 0`.

**A WRAPPER RED TEST'S REVERT WAS A SILENT NO-OP AND THE RE-TAKEN GREEN CAUGHT
IT.** The rebuild inside the restore did not recompile `Filters.f90` — the
bind-mount mtime hazard this RUNBOOK already records — so the post-integration
green came back `FAIL 9033/293` on a source `git diff` reported clean. `touch`,
rebuild, `PASS 9033/0`. Re-taking the green rather than trusting the restore is
the only reason this is a sentence and not a false artifact.

---

**As of 2026-08-14: unit #37 `PowerControlSetpoints` is `integrated` and CLOSED**
— all five layers exist, all five ran, all five are red-tested, and the mutation
score is **1.000**. But 1.000 is not the number that describes this unit. The
number is **68 of 76 mutants killed across THREE oracles**, because the unit has
three kinds of output and no single instrument sees more than one — and its
differential corpus had to be EXTENDED before it could see eleven of the unit's
twenty statements at all.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 41 cases (`evidence/PowerControlSetpoints/kernel.verify_fields.csv`) | 41/41, 18,860 field rows `IDENTICAL` | four stubs: the outer predicate inverted fails **41 of 41**; a translation reading NO input and writing four constants passes **41 of 41** |
| differential harness (`harness/PowerControlSetpoints.json`) | **3648 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **3648 of 3648**; OpenLoop's three `interp1d` calls replaced by `1.0`: **19 of 3648**, and **0 of 3596** without the R11 baseline |
| unit 401, the FORMAT (`evidence/PowerControlSetpoints/ld_probe.txt`) | **22,526 gfortran records, 0 mismatched**, against the formatter sliced out of the shipped translation | three perturbations: **488 / 8334 / 12512** |
| unit 401, the CONNECTION (`evidence/PowerControlSetpoints/f401_mutants.json`) | the reference's `fort.401` beside the translation's `fort.401.cpp`, **byte identical**, 459 bytes, md5 `d6780bd7` | three `write_401` mutants, **3 of 3 killed** |
| mutation (`mutation/PowerControlSetpoints.json`) | **68 of 76 killed across three oracles**, 8 declared, 1 no-compile, 6 operators — the harness ALONE is **0.3816**, `mutation/PowerControlSetpoints.harness-only.json` | the score *is* the red test, 68 times |
| post-integration (`harness/PowerControlSetpoints.postintegration.json`) | 3648 checked, 0 failed | the wrapper's LocalVar copy-back deleted: **3648 of 3648** |
| gate, 27 scenarios (`gate/PowerControlSetpoints.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO red tests, and their scenario sets are DISJOINT — see below |

**ELEVEN OF TWENTY STATEMENTS RAN ZERO TIMES AND THE GREEN DID NOT SAY SO.**
`probes/arm_census.cpp` is the translation with one counter per arm; it fails 0
of the corpus, so it is a reading and not a perturbation. On the first corpus:
`PRC_Mode==2` 17 of 3596, `Ind_R_Speed>0` **0**, `WRITE(401,*)` **0**, the ELSE
arm 3579. The cause is arithmetic: six predicate knobs,
4·4·4·5·4·4 = **5120**, past `generate.py::_KNOB_CASE_LIMIT` of **4096**, so R6
falls back to ALL PAIRS — and in the fallback every knob not being crossed sits
at the first value of its ladder, `PRC_Mode = 0`. Reaching the WRITE needs
`PRC_Mode=2` **and** `PRC_Comm=1` **and** `Ind_R_Speed>0` in ONE case: a TRIPLE,
which all-pairs cannot express. **1024 combinations over the bound cost this
unit its whole OpenLoop arm**, and this is the first unit in the campaign to
reach that fallback — every earlier artifact says `the full cross product`.

Closed by ADDITION: `harness/baseline.PowerControlSetpoints.json` states two
admissible states and R11 walks each knob off them. 52 cases added, 3596
unchanged. Load-bearing by measurement, not by argument: the same stub fails
**19 of 3648** with the baseline and **0 of 3596** without it.

**68 OF 76, AND THE SPLIT IS THE POINT.** 29 killed by the differential harness,
36 by the list-directed format oracle, 3 by the unit-401 file oracle. **Zero of
the eight declared survivors is in a statement transcribed from the Fortran** —
every mutant in the unit's twenty reference statements is killed by the harness.
Six of the eight are unreachable *by the type*: a finite double's decimal
exponent runs −324..308, so the `Ee` overflow arm and everything in it is dead
for any input C admits (`ld_survivor_census.txt`: body length equals field width
in **0 of 22,526** records; the stripped exponent is never `"0"` and never
longer than 3 digits).

**`equivalent_declared: 47` OVERSTATES ITS OWN FIELD NAME, SAID HERE RATHER THAN
LEFT TO BE FOUND.** `vit_mutate.py` has one bucket for "not killed by me"; 39 of
those 47 are KILLS by a named oracle with a named mismatch count, each written
out in `mutation/PowerControlSetpoints.equivalences.json`. A `killed_by` field
is a candidate for the Driver.

**C12 — A MUTATION SCORE AND THE GREEN IT IS SCORED AGAINST MUST NAME THE SAME
CASE COUNT.** The first sweep was complete, internally consistent and entirely
wrong: `vit_mutate.py` re-runs the harness against whatever case file is on
disk, and the last thing to write it was the P10 control that runs WITHOUT the
R11 baseline — 3596 cases, the corpus that never enters the OpenLoop arm. Score
0.2763, and nothing in it naming which corpus it read. It was caught because its
survivor list reproduced the arm census's zeros exactly. Kept at
`evidence/PowerControlSetpoints/mutation.WRONG-CORPUS-3596.json`. Nothing in the
pipeline checks this: `mutation/<U>.json` records `compared_against` and no case
count at all.

**THE GATE NEEDED TWO RED TESTS BECAUSE THE UNIT'S TWO LIVE ARMS ARE DRIVEN BY
DISJOINT SCENARIO SETS.**

```
ELSE arm,      PRC_Min_Pitch = PC_FinePit + 0.01   1,781,601 of 5,252,000
                                                    129 channels, 21 scenarios,
                                                    NOT scenario 25
PRC_Mode==2,   PRC_R_Speed * 1.000001                  22,660 of 5,252,000
                                                    6 channels, ALL scenario 25
```

Both revert-verified at 0 of 5,252,000. Scenario 25 is the only one of the 27
with `PRC_Mode=2`, so either red test ALONE would have read as "the gate sees
this unit" while being blind to half of it.

**A PROBE THAT GOES RED MUST SURVIVE ITS OWN `set -e`.** `run_ld_probe.sh`
measured correctly and echoed nothing: a red probe exits 1, `set -e` killed the
script one line before its own `cat "$OUT"`, and **37 of 45 formatter mutants
were first graded `nocompile` when every one was a kill**. The artifact on disk
was right the whole time; only the echo was lost, which is the worse shape — the
file and its reader disagreed and the file was the one nobody read.

---

**As of 2026-08-14: unit #36 `PitchSaturation` is `integrated` and CLOSED** —
all five layers exist, all five ran, all five are red-tested, and the mutation
score is **1.000** on 9 behavioural mutants with **6 declared equivalent**. It
is the campaign's most strongly gate-visible unit — a 0.05 rad offset on its
return value moves **1,976,629 of 5,252,000** compared values, 38% of everything
the gate compares — and its differential harness needed **three separate
repairs, one of them to the generator itself**, before it could produce a number
at all.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases (`evidence/PitchSaturation/kernel.verify_fields.csv`) | 62/62, 16,306 field rows `IDENTICAL` | seven stubs: only **4** of them can fail, and the 3 that fail all pass the same 21 cases |
| differential harness (`harness/PitchSaturation.json`) | **1036 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **1036 of 1036** |
| mutation (`mutation/PitchSaturation.json`) | **9 of 9 behavioural, 1.000**, 6 declared equivalent, 0 no-compile, 6 operators — **0.600 UNDECLARED**, `mutation/PitchSaturation.undeclared.json` | the score *is* the red test, 9 times |
| post-integration (`harness/PitchSaturation.postintegration.json`) | 1036 checked, 0 failed | the wrapper's LocalVar copy-back deleted: **1036 of 1036** |
| gate, 27 scenarios (`gate/PitchSaturation.json`) | 5,252,000 values / 351 channels, 0 mismatched | the return value +0.05 rad moves **1,976,629 of 5,252,000**, revert-verified |

**THE DIFFERENTIAL HARNESS FAILED 1292 OF 1292 AND THE CAUSE WAS THE
GENERATOR.** `expand_derived` prefers a real Fortran field as an allocatable
array's extent — `PS_BldPitchMin_N` for `PS_BldPitchMin` — while VIT's view
struct carries `n_PS_BldPitchMin` and the bridge-call emitter passes
`&<inst>.n_PS_BldPitchMin` unconditionally. Nothing assigned it. The reference
allocated **zero** elements and the translation read an extent of **0**, so the
C++ side interpolated over a table it believed had no entries. Fixed in
`translation-loop e54bef7`, one guarded line; the failing artifact is kept.

```
1292 of 1292   the view struct's extent member never assigned     <- generator
 136 of 1292   the two extents drawn independently                <- ranges.toml
  16 of 1292   the R13 capacity ladder, and 16 == len("PitchSaturation:")
   0 of 1036   the number
```

**EXPOSURE OF THAT DEFECT TO UNITS ALREADY CLOSED, STATED RATHER THAN
ASSUMED.** Five fields in this campaign's types have a companion count —
`WE_CP`, `WE_FOPoles`, `PS_BldPitchMin`, `SU_LoadStages` in `ControlParameters`
and `ACC_INFILE` in `LocalVariables`. Two committed translations read one of
their extents, and one of the two is **`CheckInputs`**: its `n_SU_LoadStages`
was 0 in every case, so `any_lt(CntrPar->SU_LoadStages, ...)` was unreachable in
both its harness green and its mutation score. Reported, not fixed here.

**THE LAST 16 ARE AN ARITHMETIC IDENTITY, NOT A RESIDUE.** `len("PitchSaturation:")`
is 16, and the failures are the 16 consecutive capacities in [14, 29] — where
`interp1d`'s prefix fits and this unit's does not. The pre-integration reference
gates `ErrMsg` **twice**, through two different buffers: `interp1d` is already
integrated, so it re-enters C++ through its own bridge over the module staging
buffer, and the case's stated capacity is applied only once, at export. The
shipped build gates once — one `ErrVar_view` in the wrapper, one direct
`interp1d_c` call in the `.cpp`, checked in the source rather than claimed.
`R13_staging_capacity` is **ablated** for the scored corpus and the cost is
named: the refusal boundary of `assign_errmsg` is unreachable here.

**VIT'S OWN KERNEL RED TEST RECORDED `demonstrated` HAVING NEVER RUN A PERTURBED
KERNEL.** `redtest.py` rewrites *every* `return <expr>;` in the file, this
translation's `errmsg_trim` returns a `std::string`, the perturbed file does not
compile, and `_run_red_test` counts a build failure as "a red of the crudest
kind, and an honest one" — then returns before trying the value-level
perturbation. Unit #10's finding mirrored, landing in a field named `red_test`.
The two false lines are removed from `vit.yaml` with the reason written where
they were, and the kernel's discriminating power was measured by hand instead.

**THE SAME SEVEN STUBS THROUGH BOTH INSTRUMENTS, WHICH IS WHAT SEPARATES A
BLINDNESS FROM AN EQUIVALENCE.**

```
                          kernel   harness
                           of 62   of 1036
noop                          41      1036
wrong-constant                62      1036
unfiltered-wind               40       249
no-ps-min-pitch               40      1036
no-max                         0       192   <- observable; the kernel cannot see it
max-swapped                    0         0   <- EQUIVALENT, and now measured
no-errmsg                      0      1004   <- observable; NO simulation instrument sees it
```

`PRC_Min_Pitch` is 0.0 in all 62 kernel cases and `aviFAIL` is 0 in all 62;
`coverage/line_coverage.json` says the same of the whole simulation —
`ControllerBlocks.f90:543`, **0 hits in 27 scenarios** against 391,977 on the
three statements around it.

**SIX OF NINE SURVIVORS DECLARED, AND WHERE THEY ARE IS THE ANSWER TO WHETHER
SIX IS TOO MANY.** This unit is three statements carrying six mutants; **five
are killed** and the sixth is the `max`'s argument order, commutative and
measured at 0 of 1036 on a corpus that includes negative zero. The other five
are at sites in the two helper functions **copied verbatim from
`interp1d.cpp`**, and four of the five carry a standing declaration from an
earlier unit at the same site for the same reason — the capacity guard for the
**fifth** time. Each is measured by a counter probe run GREEN over all 1036
cases: 1004 calls to `errmsg_trim`, `n_ErrMsg` 9..19, cap 4097..4106, longest
message 35, and **zero** at `n == 1`, `n <= 0`, `s.size() == cap` or a trailing
blank.

**AND TWO OF THOSE ZEROS ARE STRUCTURAL, WHICH IS STRONGER THAN A COUNT.**
`errmsg_trim` runs only under `IF (aviFAIL < 0)`; `aviFAIL` is negative there
only because `interp1d_c` already ran; so interp1d's own tail has already left
`"interp1d:" + TRIM(...)` in the field — at least nine characters, and with the
trailing blanks removed. A one-character message and a trailing blank cannot
reach this line **while the callee remains interp1d**. `interp1d` kills both of
those mutants at the identical source line. The blindness belongs to the callee.

**K3: FIVE EVIDENCE ARTIFACTS ACROSS FOUR UNITS WERE SILENTLY UNTRACKED.**
`.gitignore:65`'s `*build*` is upstream ROSCO's and matches by filename anywhere
in the tree, so an evidence file whose name contains "build" is dropped by
`git add -A` with no output. All five that existed record a FAILURE — including
the transient build failure this file tells the story of for #35, which until
now existed on one machine only. Closed by addition: three negations under
`evidence/`.

---

**As of 2026-08-14: unit #35 `PIIController` is `integrated` and CLOSED** — all
five layers exist, all five ran, all five are red-tested, and the mutation score
is **1.000** on 30 behavioural mutants with **nothing declared equivalent**, on
the first dispatch. It is the campaign's third `piParams` sibling after #33 and
#34, and the first whose KERNEL is alive, green and blind.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 20 cases (`evidence/PIIController/kernel.verify_fields.csv`) | 20/20, 4780 field rows `IDENTICAL` — **and the 20 cases are all-zero data** | nine stubs and one probe: only **3** of them can fail |
| differential harness (`harness/PIIController.json`) | **4528 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **4528 of 4528**, naming five outputs |
| mutation (`mutation/PIIController.json`) | **30 of 30 behavioural, 1.000**, 0 declared equivalent, 1 no-compile, 7 operators | the score *is* the red test, 30 times |
| post-integration (`harness/PIIController.postintegration.json`) | 4528 checked, 0 failed | the MERGE inverted **4528 of 4528**; the two errors transposed **164 of 4528** |
| gate, 27 scenarios (`gate/PIIController.json`) | 5,252,000 values / 351 channels, 0 mismatched | the raw sum +1000.0 moves **11,997 of 5,252,000** in 3 channels, revert-verified |

**THE KERNEL PASSES 20 OF 20 AND CANNOT SEE NINE OF THE TEN THINGS THE UNIT
DOES.** Nine one-edit stubs generated from the shipped translation, plus one
probe, run through the committed kernel:

```
the whole unit as a no-op returning 0.0     fails objInst%instPI ONLY
the determinate wrong constant -7.25        fails Flp_Angle AND instPI
`inst = inst + 1` deleted                   fails instPI
the ITerm / ITerm2 / output clamp deleted   PASSES 20/20, each
piP%ITermLast(inst) = ... deleted           PASSES 20/20
`error` forced to 0.0                       PASSES 20/20
`error2` forced to 0.0                      PASSES 20/20
a wrong answer if EITHER input is non-zero  PASSES 20/20   <- the one that closes it
```

Both error inputs are identically zero in every captured case, so the reference
returns 0.0 twenty times out of twenty. Unit #2's ColemanTransform shape at a
different call site. **The capture size was predicted before extracting** by
unit #25's arithmetic — the window is 62 slots, the site is called 11,994 times,
so ranges two and three are past its last call and 20 is the whole of what the
kernel could hold. It came back 20.

**AND VIT'S OWN RED TEST WENT RED ON THE INPUT THE STUB SHOWS IS DEAD.** `vit
verify` reported *"a mismatch with input `error` offset by 1e-05"*, which is
true — offsetting a zero makes it non-zero — and is a claim about the
INSTRUMENT, where the stub is a claim about the CORPUS. Unit #33 measured a
refusal's stated reason because a refusal can be wrong; this is the mirror, a
demonstration whose stated reason is right and is not the claim a reader would
take from it.

**A GATE RED TEST WHOSE COUNT IS THE COVERAGE'S CALL COUNT, TO THE BLADE.** The
raw sum offset by +1000.0 moves 11,997 values across `scenario_4:flp_angle_1`,
`_2` and `_3`, each 3999 of 4000. `coverage/line_coverage.json` independently
records 11,997 calls, all in scenario 4, decomposing as 1 + 3998 per blade.
gcov's statement counter and a bit-exact comparison of simulation output agree
on the total and on the decomposition. The perturbation is contained to those
three channels because the output is clamped to ±`Flp_MaxPit` and fed straight
back as the next call's `error2`; `perturbation_broke_scenarios` is empty, so
these are values moved and not values lost.

**P10 CAUGHT A COLLAPSED COMPARISON BEFORE IT PRINTED A NUMBER.** Three counting
probes were generated; before running any of them the no-op was re-run through
the same path as a control, and on the integrated tree it fails **0 of 4607**.
Two things had moved: `harness.sh` links `vit_integration_shim.o`, so after
integration the wrapper IS the reference and both sides run the harness's own
copy (unit #29's finding, reproduced) — and the corpus changed size, because the
generator mines the reference's literals and a marshalling wrapper has none. The
probes were re-run on the third tree state, this unit reverted and every other
integrated, reached by `git checkout` on three paths with an EXIT trap.

```
P10 CONTROL, the no-op on that tree        4528 of 4528
the ELSE arm reached                       2261 of 4528
the RESET arm reached                      2267 of 4528
                                           ----
                                           4528
```

**THE UPSTREAM ASYMMETRY IS OBSERVABLE, WHICH IS WHY TRANSCRIBING IT IS A
DECISION.** The reference writes `piP%ITermLast(inst)` in the ELSE arm and does
not write `piP%ITermLast2(inst)` there, although the reset arm initialises both.
The no-op does not name `ITermLast2` among the five outputs it moves, and that
absence alone is indistinguishable from an unreachable channel. Adding the
mirror write is rejected on **2261 of 4528** — every ELSE-arm case.

**THE THREE `saturate_c` SITES STILL GET NO MUTANT, AND ONE HAND-RUN SHAPE
EXISTS AT NO UNIT.** `evidence/PIIController/hand_mutants.txt`, baseline 0 of
4528:

```
drop_call    ITerm / ITerm2 / output clamp     2250 / 2254 / 2013
             both integrator clamps            2255
swap_args    value <-> minValue, all three        0 /    0 /    0   EQUIVALENT
transposed   bounds swapped, all three          117 /  290 /  199
swap_callee  the two integrator clamps exchanged             14
```

The first two rows say the corpus does **not** separate the two integrator
channels through their clamps — three counts within five of one another. The
last row is what does separate them, and `swap_call_args` cannot produce it
because it exchanges arguments *within* a call. The three zeros are equivalences
already proved at `evidence/saturate/minmax_probe.txt`, so the standing
amendment's effect here is +3 kills, +3 equivalences and a score unchanged at
1.000 — free for the third consecutive unit.

**NO ENTRY WAS ADDED TO `harness/ranges.toml`, AND THE ABSENCE IS A DECISION.**
This unit has PIDController's structural gap — `minValue` and `maxValue` drawn
independently from one default — and it cost nothing measurable here. A pin
narrows a domain and every entry in that file carries the measurement that
forces it; there is none to carry. `R15_bracketing_bounds` remains the
generator-level remedy and remains unbuilt.

**TWO INSTRUMENT FAULTS, EACH RECORDED WITH ITS FAILING ARTIFACT FIRST (C12).**
`scripts/harness.sh` could not run its own documented usage line — `${ARGS[*]}`
on an empty array is unbound under `set -u` in bash 3.2, latent for thirty-four
units because every previous pre-mode invocation passed `--out`. And a transient
build failure at the harness's own build step **deleted the committed
post-integration green**, after which `git add -A` committed that deletion under
a message saying the green had been re-taken (`b090ab2`, left unamended).
`harness.sh` redirects into `--out`, so a died build destroys what was there;
making it write-then-rename changes how every artifact in this campaign is
written, which is X3's question and not this unit's.

---

**As of 2026-08-14: unit #34 `PIDController` is `integrated` and CLOSED** — all
five layers that exist for it ran, every green is red-tested, and the mutation
score is **1.000** on 29 behavioural mutants with **one** declared equivalence.
It is the campaign's **fourth dead unit** (#1 `AddToList`, #21 `UpdateZeroMQ`,
#26 `unwrap`) and the first whose deadness the campaign's own scenario set
*tried* to prevent.

**THE SIX SURVIVORS OF THE FIRST DISPATCH WERE UNREACHABLE, NOT EQUIVALENT, AND
NOT ONE OF THEM WAS DECLARED.** The score moved 0.759 → 0.966 by changing the
INPUTS, and only then 0.966 → 1.000 by declaring the seventh, which is an
absence rather than a weakness. Declaring all seven would have reached 1.000 in
one edit and verified nothing.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT AVAILABLE** — one call site, 0 hits in all 27 scenarios | `evidence/PIDController/coverage_deadness.txt`, exit 0, with a positive control at 12,339,878 hits |
| differential harness (`harness/PIDController.json`) | **9758 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **9758 of 9758** |
| mutation (`mutation/PIDController.json`) | **28 of 28 behavioural, 1.000**, 1 declared equivalent, 1 no-compile, 7 operators — and **0.966 UNDECLARED**, `mutation/PIDController.undeclared.json` | the score *is* the red test, 29 times |
| post-integration (`harness/PIDController.postintegration.json`) | 9758 checked, 0 failed | the MERGE inverted **9758 of 9758**; the copy-back removed **9758 of 9758** |
| gate, 27 scenarios (`gate/PIDController.json`) | 5,252,000 values / 351 channels, 0 mismatched | the return value +1000.0 moves **0 of 5,252,000** — `RED_TEST_FAIL`, revert-verified |

**THE SCENARIO BUILT FOR THIS UNIT IS ONE OF THE THREE A MISSING FILE STOPS.**
`vit_sim.py`'s scenario 10 exists for exactly this function — its docstring is
*"OL_Mode=2 azimuth tracking to exercise PIDController"* — and sets `OL_Mode=2`,
`Ind_GenTq=5`, `Ind_Azimuth=6`, `RP_Gains = 1000 100 500 0.1`. It is also one of
scenarios 10/14/24, which reach `ReadSetParameters.f90:778`, fail in
`Read_OL_Input` on the absent `Examples/example_inputs/OL_Mode2_Input.dat`
(unit #17), and are taken out by the `RETURN` two statements later. **Supplying
that file would change what 3 of the gate's 27 scenarios compute, which is what
X3 forbids mid-run**; it is the single highest-value thing the campaign could do
for its own coverage, and it is a Driver decision, not a unit's.

**WHAT THE CORPUS COULD NOT SEE, AND WHAT MADE IT SEE.** A census probe writes
one CSV row per call and changes **no compared output**, so the run that
produces it is itself GREEN — a reading of the corpus rather than a perturbation
of it. One run replaces one bit-per-case counting probe per question
(`evidence/PIDController/make_census.py`, `census_report.py`,
`clamp_census.{before,after}.csv`). Counts are over the ELSE arm:

```
                                          BEFORE            AFTER
the return value == a bound          2307 of 2307      3568 of 4884
minValue < maxValue AT ALL            207 of 2307      4884 of 4884
EFilt finite                            1 of 2307      1507 of 4884
the OUTER clamp is INACTIVE             0 of 2307      1316 of 4884
the ITerm clamp is INACTIVE             8 of 2307      4285 of 4884
```

**TWO INDEPENDENT CAUSES, AND FIXING EITHER ALONE LEAVES THE ZERO AT ZERO.**
`LPFilter` divides by `FP%lpf1_a1(inst)` (`Filters.f90:65`), which is written
only inside `IF ((iStatus == 0) .OR. reset)` — and the harness supplies
`LocalVar%FP` **zeroed** on every case, so `EFilt` is `1.0/0.0 * 0.0` = NaN
wherever `iStatus /= 0`, and NaN swamps the sum. Separately, `minValue` and
`maxValue` are drawn INDEPENDENTLY from one default and came out `(0, -300)` in
**1512 of 2307** cases; R6's isolating stage sets every *other* defaulted real
EQUAL, which is the same empty interval by another route. `saturate(x, lo, hi)`
is `MIN(MAX(x,lo),hi)` and returns `hi` for every x once `lo >= hi`.

**THE REMEDY IS THREE ENTRIES IN `harness/ranges.toml` AND NOT A GENERATOR
CHANGE** — `R15_bracketing_bounds` in `DECISIONS.md` still specifies that, and
X3 still forbids it mid-run. `LocalVar_iStatus` takes the reference's own
documented enumeration `[0, 1, -1]` with 0 first; `minValue` takes
`[-1e9, -1e-3]` and `maxValue` `[1e-3, 1e9]`, which is the only call site's own
shape (`Controllers.f90:346` passes `-LocalVar%VS_MaxTq*2` and
`+LocalVar%VS_MaxTq*2`) widened, with the magnitude taken from the clamped sum
itself (|raw| median 2994, p90 1.86e6). Ranges are read per unit, so **no other
unit's corpus moves**, and `loop_rev` is unchanged at `e7d5583` across all nine
result artifacts (`revcheck --unit PIDController`: clean).

The six then die at ordinary counts — `kp*error -> kp` 1288, `-> kp/error` 1349,
`EFilt-ELast -> EFilt+ELast` 1367, `PTerm+ITerm -> PTerm-ITerm` 1325, and the
two `[i] -> [i+1]` reads at lines 112 and 121 at 1274 and 1323.

**THE ONE DECLARED EQUIVALENCE IS AN ABSENCE, AND THE MUTANT BESIDE IT IS THE
PROOF.** `const_tweak '0.0' -> '1.0'` on the last argument of
`lpfilter_c(..., 0, 0.0)`: the preceding literal `0` is `has_InitialValue`, the
generated bridge does not pass the argument at all in that arm, and the shipped
C++ `LPFilter` reads it only under `if (has_InitialValue)`. The **flag's own**
mutant, one position left on the same line, is **killed in 1145 of 9758** — so
the corpus distinguishes the gate and fails to distinguish only the value the
gate makes unreadable. `mutation/PIDController.equivalences.md`.

**TWO INSTRUMENT DEFECTS HAD TO BE FIXED BEFORE ANY LAYER COULD RUN**, both
recorded with their failing artifact first (C12), and both the same family — a
helper whose notion of a name is a bare identifier:

```
vit  3ce00e8   the kernel callee header names a derived-type dummy and never
               included vit_types.h; g++ recovers by reading it as `int`
loop e7d5583   neither index inferrer could see an index that is ITSELF a
               derived-type field. instPI outside 1..1024:
                   4741 of 4771 cases BEFORE     0 of 4692 AFTER
```

The second's symptom was `harness produced no JSON` — which is also what a
printing reference produces — because the REFERENCE was indexing a
`DIMENSION(1024)` array with -300 and with 2147483647.

**AND THE UNIT'S SECOND INDEX NEEDED THE OPPOSITE ANSWER.** `objInst%instLPF` is
subscripted inside the CALLEE, against a field of a NESTED type the generator
does not expand — no compared out-parameter exists to attach a role to, so no
inference can ever reach it and it takes the campaign's only judgement mechanism,
`harness/ranges.toml`, backed by the reference's own exit status
(`evidence/PIDController/instlpf_probe.txt`: SIGSEGV in `__filters_MOD_lpfilter`
at both 32-bit extremes and at -100000).

**WHAT IS STILL NOT SEEN.** `LocalVar%FP` is a NESTED type the generator
zero-initialises and never varies, so `LPFilter`'s RECURSIVE arm is unreachable
from here — 3377 of 4884 else-arm cases still answer NaN, and what the corpus
exercises is the INIT arm plus NaN propagation. That is a generator gap, not
something a range can state; it is in `DECISIONS.md`. An INVERTED bracket
(`lo > hi`) is also no longer generated, and the defect that would need it lives
in `saturate` — unit #24, CALLED not inlined, so both sides of this comparison
always reached one implementation of it and this harness never constrained it.

---

**As of 2026-08-14: unit #33 `PIController` is `integrated` and CLOSED** — all
five layers ran, all five are red-tested, and the mutation score is **1.000** on
21 behavioural mutants. It is the campaign's first unit with a TRANSLATED CALLEE
in its body (`saturate`, unit #24), and the first whose kernel blindness was
predicted by VIT's own red test before a stub measured it.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases (`evidence/PIController/kernel.verify_fields.csv`) | 62/62, all 14,508 field rows `IDENTICAL` | six stubs: 62/62 · 61 · 61 · 62 · 39 · **17** |
| differential harness (`harness/PIController.json`) | **3532 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | the unit as a no-op: **3532 of 3532**, naming all four outputs |
| mutation (`mutation/PIController.json`) | **21 of 21 behavioural, 1.000**, 0 declared equivalent, 1 no-compile, 7 operators | the score *is* the red test, 21 times |
| post-integration (`harness/PIController.postintegration.json`) | 3532 checked, 0 failed | bounds transposed **145 of 3532**; the MERGE inverted **3112 of 3532** |
| gate, 27 scenarios (`gate/PIController.json`) | 5,252,000 values / 351 channels, 0 mismatched | whole-unit no-op **1,780,508**; `error` zeroed **2,133,598**; reset inverted **2,128,633** |

**THE KERNEL IS 45-OF-62 BLIND TO `error`, AND VIT SAID SO FIRST.** `vit verify`
printed `the input perturbations were absorbed (input \`error\` scaled by 1.00001,
input \`error\` offset by 1e-05) ... typically a saturated output` beside its
62/62. That is a refusal to claim, and this campaign's rule is that a refusal's
stated reason is a claim of its own (unit #32) — so it was measured. The
reference return value equals `minValue` in **62 of 62** captured cases: at
Controllers.f90:68 the pitch loop is saturated at its lower bound throughout
scenario 1's window, so the RETURN carries nothing about `error`, and a stub that
forces `error` to zero **passes 45 of 62**. What the 17 failures see is the
`piP%ITerm` state the clamp lets through.

It is closed by two other oracles rather than left open: the harness varies
`error` over 3532 cases, and the same edit made at the GATE moves **2,133,598 of
5,252,000** values — *more* than a whole-unit no-op moves, because an error-blind
unit keeps clamping and keeps advancing `inst`, so the controller stays in the
loop and the trajectories diverge further.

**READ THE STATUS COLUMN, NOT THE VALUE COLUMNS.**
`evidence/PIController/kernel.verify_fields.csv` prints `instpi` as `3,3` in all
62 rows while the stub that deletes `inst = inst + 1` moves that exact field in
all 62 cases. The CSV also carries 14,508 rows where the kernel's own stdout
carries 14,818. The verdicts agree; the value columns are not the compared
values, and a reader who quoted them would report the increment as
unconstrained.

**THE MUTATOR'S 22 MUTANTS TOUCH NEITHER `saturate_c` CALL.** All three of
`cppmutate`'s call operators are gated on TABLES of callee names
(`_VALUE_PRESERVING`, `_SIBLINGS`) and every entry is a C standard-library name.
The artifact records this itself — `operators_offered` lists twelve,
`operators` lists seven — so the gap is legible without reading the mutator.
Adding `saturate_c` is a plausible amendment and a campaign-wide re-take, which
X3 forbids mid-run, so the measurement was made BY HAND exactly as unit #24 made
it before the operator existed (`evidence/PIController/hand_mutants.txt`,
baseline 0 of 3532):

```
drop_call    ITerm clamp dropped                  1761 of 3532
             output clamp dropped                  383 of 3532
swap_args    ITerm clamp,  value <-> minValue         0 of 3532
             output clamp, value <-> minValue         0 of 3532
transposed   ITerm clamp,  bounds swapped           63 of 3532
             output clamp, bounds swapped          135 of 3532
```

The two zeros are not a corpus gap: `fmax(a,b)` and `fmax(b,a)` agree on every
input including a signed zero and a NaN, already proved on this toolchain at
`evidence/saturate/minmax_probe.txt`. Had the operator fired it would have added
two kills and two equivalences and left the score at **1.000**.

**ONE PREDICATE, THREE INSTRUMENTS, ONE NUMBER.** The reset branch was perturbed
from the C++ side (`negate_cond`, 3112 of 3532), from the Fortran WRAPPER side
(`MERGE(1_C_INT, 0_C_INT, reset)` inverted, 3112 of 3532) and from the whole
program (the gate, 2,128,633 of 5,252,000). The first two agreeing to the case
is what says the predicate is constrained rather than merely covered; the third
is what says a branch taken once per PI instance per scenario, against ~1.44M
ordinary calls, is not invisible to simulation output.

**AND ONE FALSE FINDING WAS CAUGHT BEFORE IT WAS WRITTEN.** This unit has 17
call sites and `coverage/line_coverage.json` reports **0 hits** at the line of
the CableControl one. It is not dead: gcov attributes a continued statement's
hits to its LAST continuation line, and line 947 carries 127,994. Four of the 17
sites read as dead at their own line and none of them is. Check the continuation
before recording a dead call site.

---

**As of 2026-08-14: unit #32 `FindLine` is `deferred` and NOT CLOSED** — the
mutation score is an honest **0.960** against a threshold of 1.000, on **one**
survivor. Its first dispatch closed at 0.760 with six, five of which were
instances of ONE measured corpus gap; this dispatch built the rule that gap
specified and all five died. The one that is left is undefined behaviour, is
classified `(c)`, and is escalated. Everything else this unit has ran and is
green, and every green is red-tested. It is the campaign's first unit taking an
**assumed-shape CHARACTER array**, and that one declaration broke three tool
defects loose.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 20 cases (`evidence/FindLine/kernel.verify_fields.csv`) | 20/20, all 60 fields `IDENTICAL` | four stubs: 0/20 · 0/20 · **20/20** · 5/20 |
| differential harness (`harness/FindLine.json`) | **2514 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | seven stubs: 2511 · 47 · 62 · 89 · **30** · **42** · 60 |
| mutation (`mutation/FindLine.json`) | **24 of 25 behavioural, 0.960**, 2 declared equivalent, 0 no-compile, 9 operators | the score *is* the red test, 25 times |
| post-integration (`harness/FindLine.postintegration.json`) | 2514 checked, 0 failed | the wrapper's two extents transposed: **88 of 2514**, revert-verified 0 |
| gate, 27 scenarios (`gate/FindLine.json`) | 5,252,000 values / 351 channels, 0 mismatched | the name match disabled moves **1,857,893**, revert-verified 0 |

**THE GAP WAS 47 OF 2370 AND IT IS NOW 89 OF 2514, OF WHICH 42 ARE NOT BLANK.**
`evidence/FindLine/findline.match-count-probe.cpp` writes `LineNum = -7` inside
the match arm — `LineNum` is compared by R4 and the reference writes only 0 or
`I`, so `-7` cannot collide with a real answer — and its failing count *is* the
number of cases that find the name. `findline.nonblank-key-probe.cpp` refuses
the match unless the key holds a non-blank character and fails 47, so the
blank-vs-blank matches are still there and **42 new ones are not**. Before this
dispatch the second number was **0**: every match in the whole corpus was two
empty strings comparing equal, which cannot tell one word index from another,
cannot see an uppercasing pass, and cannot tell a width of 200 from 201.

**THE RULE IS R14_planted_word (`translation-loop` `552edb1`)**, and it is
almost verbatim the rule the first dispatch wrote down and declined to build. It
plants a free scalar CHARACTER input as the *k*-th blank-delimited WORD of one
element of a CHARACTER ARRAY input, for *k* in 1..3, with every free scalar
integer set to *k-1* and to *k* (the generator cannot know which integer picks
the word, only that one usually does), with the key **case-inverted** against
the word so a fold dropped on either side breaks the match, with the element
FIRST and LAST and nothing else, and with the whole construction carried to
R12's narrowing width. Five survivors died on five different counts — 14, 48,
18, 21 and 72 — which is the evidence that they were five behaviours and not one
seen five ways. The two stubs that used to report the blindness as a zero now
report **30 of 2514** and **42 of 2514**, and that 42 is exactly the non-blank
match count, which is what it has to be.

**ITS X3 COST IS A BYTE-PREFIX IDENTITY, NOT A CASE COUNT.** R14 is appended
after every other rule, so the claim is that an already-scored corpus is
strictly extended, and `evidence/FindLine/x3_check_r14/` proves it on all three
units that can fire it: `ChkParseData` 102,577 → 110,533 bytes with the first
102,577 identical, `GetWords` unchanged (its array is `INTENT(OUT)`, so the rule
reports N/A), `FindLine` 4,990,604 → 5,370,716 with the first 4,990,604
identical. The R14-off column reproduces the committed case counts exactly, so
the ablation is a real one. **That property cost R12 the same claim** and the
loop's test says so rather than being made to pass: only one rule can be last.

**THE ONE SURVIVOR IS `(c)`, AND BOTH OF ITS ZEROS HAVE A CONTROL AT THEIR OWN
SITE.** `'2048' -> '2049'` on `MaxLineLength` makes `char_assign` write one byte
past the *caller's* 2048-byte buffer.

```
                                  the mutant                the control, same site
differential harness, 2514 cases  0 failed                  2048 -> 2047:  60 failed
gate, 5,252,000 values            0 moved, 0 channels,      2048 -> 5:     1,583,216 moved,
                                  0 scenarios broken        131 channels, and scenarios
                                                            19 and 27 stopped running
```

It is **not (a)**: the two programs do not agree on every admissible input,
because one of them has no defined behaviour at all (unit #7). It is **not
(b)**: no widening of any corpus reaches it, because the difference is not in
any compared value — now measured against a second, whole-program oracle rather
than inferred from the first. The site cannot be removed either: VIT emits no
`len_Line`, so the width must be stated in the C++ exactly once. One byte too
FEW is a wrong answer; one byte too MANY is not an answer at all. Third instance
in the campaign after `Read_OL_Input` and `Debug`'s `c3a5bb71`; the instrument
is a sanitiser build and is already a **proposed method amendment**.

**AND THAT AMENDMENT IS NOW DEMONSTRATED RATHER THAN ARGUED**
(`evidence/FindLine/asan_demo/`). Under `-fsanitize=address` the shipped
translation writes **0 bytes** of diagnostic and the mutant writes a
`heap-buffer-overflow`, `WRITE of size 1`, `0 bytes after 2048-byte region`,
naming `char_assign` and the buffer the harness allocates. Three units have
carried this proposal on an argument; the two questions the argument could not
answer — does it fire, and is it silent on the correct program — are answered.
The one it still cannot answer is whether the other 31 translations are clean
under it, which is a sweep and a dispatch of its own. **The score stays 0.960**:
changing the mutation instrument for one unit is what X3 forbids.

**THE TRANSLATION LOST TWO SITES TO THE SAME RULE, AND THE SCORE MOVED
0.667 → 0.704.** `char_assign`'s `n = std::min(len_src, len_dst)` plus two
loops became ONE loop bounded by `len_dst` with an `i <= len_src` predicate, and
`std::max(WordInd, 0)` was deleted outright. Five survivors went with them —
every one computed the right answer and differed only in memory the comparison
cannot read. The rewrite is also the safer program: the loop cannot leave `dst`.

**THREE TOOL DEFECTS, EACH FIXED WHERE IT LIVES (X2), and one of them had a
FALSE REASON attached.**

1. `vit check` scoped its Fortran-reading checks to the **file**, not the
   procedure (vit `c4eb0ad`). `delimiter-set` reported `FindLine` — which
   contains no `SCAN`, `INDEX` or `VERIFY` at all — as missing the backslash
   from `GetPath`'s `INDEX( GivenFil, '\', BACK=.TRUE. )` 900 lines away.
   Latent for eight units in that one file, because `GetPath` and `GetRoot`
   happened to carry the same separators.
2. **VIT's two generators disagreed about extent ORDER** (vit `d2de28c`).
   `build_c_params` emits `char* X, int n_X, int len_X`; the test-validate
   bridge emitted `len_X` then `n_X`. C linkage checks nothing, so a 4×3
   CHARACTER array was read as 3×4 — a well-formed array of the wrong shape —
   and **2311 of 2358 cases still agreed**. The reference's `LineNum` was
   exactly `len_FileLines` in every one of the 47 that did not. The generator
   now asks `build_c_params` and refuses when they differ.
3. The loop's mapper refused the shape **with a reason that was false** (loop
   `9eeaf3f`): "carries its extent in a descriptor `build_c_params` does not
   emit", when it is emitted under the ordinary `n_` name. Two more in that
   commit — `int32_t*` looked up as a view struct because `int32_t` ends in
   `_t`, and `CHARACTER(MaxLineLength)`, this unit's principal output, reported
   `UNOBSERVABLE` because its width is a module PARAMETER rather than a literal.
   R12_narrowing_width had the same literal-only blind spot (loop `024982b`).

**THE R12 WIDENING KILLED NOTHING ON ITS OWN** — 0.667 before and after, +12
cases — and became three kills only once `char_assign` was rewritten, and a
fourth (`'200' -> '201'`) only once R14 put a 200-character WORD at that width.
A rule that puts a boundary in the corpus and a translation with no site at
which that boundary changes an answer are two halves of one measurement.

**A RED TEST THAT CORROBORATES ANOTHER UNIT'S.** Disabling FindLine's name
comparison moves **1,857,893 of 5,252,000 across 147 channels** — the same three
figures as `gate/GetWords.redtest.json`. Not a coincidence: GetWords' red test
blanks every word, so FindLine then compares a blank word against a non-blank
name and matches nothing. Two perturbations of two units converging on one state
is what a same-build control buys, taken here for free.

**TWO CLOSED UNITS NO LONGER REPRODUCE, AND BOTH ARE LEFT AS FINDINGS.**
`GetWords` produces **1373** cases where `harness/GetWords.json` records
**1370**, and that drift predates this unit entirely (it is already 1373 at loop
`12dbaa0`). `ChkParseData` now produces **1624** where its artifact records
**1552** — 72 cases R14 added, all of which its translation PASSES and none of
which its committed 1.000 mutation score was taken over. Neither is repaired
inside this unit's dispatch, because re-taking a closed unit's evidence would
put a number in its artifact that no commit of its own explains. Both are in
DECISIONS.md for the Driver.

**AND A CONTROL THAT TOOK TWO ATTEMPTS.** The two declared equivalences enlarge
a fixed-width local by one byte; a canary in that byte reports 0 of 2514
disturbed against a control that reports 2514 of 2514. The first control for
that probe — `conv2uc_c(ParamNameUC, MaxParamLength + 1)` — was a **no-op**,
because Conv2UC writes a byte only when it is a lowercase letter and the
sentinel is `\x7f`. It reported 0, the same number as the probe, and for one run
a dead control was indistinguishable from a passing probe (P10). The control now
writes the byte unconditionally. That lesson is why the surviving mutant's two
zeros were not accepted until they had controls of their own.

---

**As of 2026-08-14: unit #31 `Debug` is `deferred` and NOT CLOSED** — the
mutation score is an honest **0.6798** against a threshold of 1.000, and the
survivors are eight named shapes rather than a scatter. Everything else this
unit has ran and is green. It is the campaign's second `respecify` unit after
#17 `Read_OL_Input`, ~490 lines, and **the only one so far whose entire
observable output is a file**.

Every count below is read from the committed artifact named in its row.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **CANNOT EXIST** — KGen compares captured STATE; this unit's outputs are bytes in a file | — |
| generated differential harness | **CANNOT BE BUILT** — the unit assigns nothing in its own signature, so the comparison set is empty | — |
| file identity, 27 scenarios + scenario 28 (`harness/Debug.postintegration.json`) | **456,086 records / 383,540,428 bytes, 0 failed** — **this unit's primary evidence** | seven perturbations: 24 / 407,976 / 21,792 / 15,999 / 16,003 records red, two at 0 |
| mutation score (`mutation/Debug.json`) | **121 of 178 behavioural, 0.6798**, 3 no-compile of 181, 0 declared equivalent, 10 operators | the score *is* the red test, 178 times |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched — **and it establishes nothing about this translation** | every DebugOutData value DOUBLED moves **0**; same-build control **1,857,893** |
| format fidelity vs gfortran (`evidence/Debug/fmt_probe.txt`) | 54 records, 4598 bytes, IDENTICAL | 1 and 26 records |

**THE GATE IS BLIND BY CONSTRUCTION, WHICH IS A STRONGER STATEMENT THAN ANY
BLINDNESS THIS CAMPAIGN HAS RECORDED BEFORE.** `Examples/vit_sim.py` builds
`baseline_arrays/*.npz` from the `avrSWAP` channels the controller RETURNS.
`Debug` writes `<RootName>.RO.dbg`, which nothing in the gate path opens, and
assigns no argument of its own signature — `ErrVar` only on `GetNewUnit`'s
unit-number-exhaustion arm, `avrSWAP` never. The unit is **called ~408,000
times across 23 of the 27 scenarios and does its job every time**: this is not
unit #27's dead branches, nor units #1/#21/#26's dead call sites. The gate reads
a different output stream. No input could change that.

**WHAT COMPARES IT IS THE FILE**, and the two archives differ in one thing:
`pre` was taken with `Debug` still Fortran and every other unit already
integrated, `post` with `Debug` integrated and nothing else changed. Record 1
carries `CurDate()`/`CurTime()` and is compared byte for byte first, falling
back to a structural check only when it differs and both sides carry the
`Generated on …` shape — **named rather than dropped**, because a line dropped
from a comparison is a line the comparison cannot report on.

**A THIRD OF THIS UNIT WAS INVISIBLE TO EVERY INSTRUMENT, AND ONE INPUT
PARAMETER CLOSED IT.** All 14 `Examples/DISCON*.IN` set `LoggingLevel = 1`, so
`IF (LoggingLevel > 1)` and `> 2` are REACHED in 23 scenarios and FALSE in all
of them. `Examples/vit_sim.py::run_scenario_28` is scenario 3's configuration
with that one parameter set to 3 — outside `scenario_order`, so no gate run and
no committed baseline moves (X3, P5). Scenario 3 is the right base because it is
the only one already setting `CC_Mode = 1` and `StC_Mode = 1`, which is what
makes the two `AddToList` loops in the dbg3 path reachable at all.

```
ES20.5E2 -> E3 on the LocalVarOutData write, 27 scenarios        0 of 408,072
the same edit, scenario 28                                  15,999 of  48,014
```

Same edit, same build, one parameter apart. And the half it opened is
byte-identical: 48,014 records covering the 159 `LocalVarOutData` assignments,
the 159-name heading table with its `CHARACTER(15)` truncations, the empty
record `WRITE(UnDb2,'(160(a20,TR5:))')` writes with no data item, the six blank
records `(/////)` writes, the `avrIndices` construction through both `AddToList`
loops, the `AvrSWAP(NNNN)` heading built from a runtime format string, the `(-)`
group that runs past its one data item, and the vector-subscripted
`avrSWAP(avrIndices)` write.

**THE MUTATION SCORE'S 57 SURVIVORS ARE EIGHT SHAPES**
(`evidence/Debug/mutation.md`): 11 enlarge a buffer or array extent, which no
byte comparison can see; **7 are the `WRITE(*,100)` status line, which goes to
unit 6 and not to a file**; 7 are the two clamps at a boundary no computed value
lands on; 5 are guards whose operand is constant in every shipped input; 3 are
`Ind - 1` → `1 - Ind` where the group extent is 1; 8 are unreachable arms of the
format helpers, **two of them genuinely equivalent and deliberately NOT declared
so the score carries them**; 3 are the `TRIM` loop, whose body never runs on a
blank-free RootName.

**THE LARGEST REMAINING GAP IS THE MUTATION CORPUS, AND IT WAS MEASURED RATHER
THAN GUESSED.** The sweep runs scenarios 27 and 28 because a mutant costs a
rebuild plus a simulation. The `DebugOutData[11] → [12]` shift, re-run over the
full 24-file corpus through the same red-test path, dies on **23,998 of 408,072
records in exactly one file, `vit_sim7.RO.dbg`** —
`DebugVar%NacIMU_FA_AccF` and `%FA_AccF` are both zero in scenarios 27 and 28.
A wider mutation corpus is *known* to raise this score. The ablation is
committed beside it: the same 40 `compare_op` mutants at LoggingLevel = 1 kill
**12 of 40** where {27, 28} kills **21**.

**THREE MUTATOR DEFECTS HAD TO BE FIXED BEFORE THE SCORE EXISTED** (X2, each in
`translation-loop`, each with its cost measured over all 31 translations rather
than argued): `compare_op` rewriting the angle brackets of a **user-declared
template** (12 of 40 unbuildable, 30%, above the 25% at which a run refuses);
`arith_op` reading `std::FILE* f`, `return *p` and **the exponent sign of
`1E-99`** as arithmetic (13 of 25 — and that last one meant the unit's two clamp
constants had *no* `arith_op` mutant at all); and `_not_arithmetic` reading
`drop_factor`'s RIGHT OPERAND as its operator, so it fired for two rules of
three. The third was found by running the sweep, not by reading the diff.

**AND THE MERGE REFUSED, CORRECTLY.** The first union of the nine sweep parts
named three different `loop_rev`s, because two of those fixes landed mid-sweep.
Six parts were re-taken so that all nine name `897d67c`. Worth recording: for a
punctuation operator `_mid` is `(unit, operator, '<', '<=', nth)`, so an id
tracks the ORDINAL and not the SITE — twelve sites moved under the first fix and
three ids changed. `scripts/dbgmutate.py` records line/before/after per result
for that reason.

**C12: `dbgcheck` REPORTED A MISMATCH `.RO.dbg3` CANNOT HAVE.** Its
`compare_first_record` assumed record 1 is always the `Generated on …` line and
failed anything else — and `.RO.dbg3` opens with `WRITE(UnDb3,'(/////)')`, so
its record 1 is EMPTY and two empty records were called different. The
assumption held for every file the instrument had seen and stopped holding the
moment a third file appeared. The wrong artifact is committed at `327ca29`
before the fix.

Two respecifications are recorded rather than left to be found. **`GetNewUnit`
is absorbed**, so this unit's only write to `ErrVar` — the unit-number
exhaustion arm — does not exist in C++. **`Int2LStr` is not called**: its only
job here is to render a format string's repeat count as decimal text so the
Fortran format parser can read it back as an integer, and a loop never leaves
the integer domain. No part of its body appears anywhere in the translation, so
this is not the inlining X1 forbids; `plan.json`'s `depends_on` for this unit is
`AddToList` alone after respecification.

---

**As of 2026-08-14: unit #30 `ChkParseData` is `integrated` and CLOSED** —
`done_check.py` reports **14 of 14** (`evidence/ChkParseData/done_check.txt`).
It closed at the second dispatch; the first left it `deferred` at an honest
**0.852** on P12, and what moved it to **1.000** was two rules added to the
corpus generator, not one line of the translation and not one new equivalence
declaration. `revcheck --unit ChkParseData` is **clean**: all six result
artifacts name `f55b26f`, and every one of them was re-taken to get there —
twice, because the instrument moved twice.

Everything that CAN run for this unit ran and is green. What cannot run is two
of the five layers, and the reason is the same one: **the unit is dead in all 27
scenarios**, the fourth such unit after #1, #21 and #26. That is unchanged and
no corpus rule can change it.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **DOES NOT EXIST** — no live call site to extract at | — |
| differential harness (`harness/ChkParseData.json`) | **1552** checked, 0 failed, 0 inadmissible — **this unit's primary evidence** | seven stubs and two probes, below |
| post-integration harness | **1552** checked, 0 failed | the wrapper transposes the two words: **9 of 1552**, revert verified green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, **0 mismatched** | whole unit → a determinate error moves **0**; GetWords perturbed on the SAME BUILD moves **1,857,893** |
| mutation score | **27 of 27 behavioural, 1.000**, 4 declared equivalent, 0 no-compile, 8 operators | the score *is* the red test, twenty-seven times |

**AND THE RED TEST'S OWN REVERT WENT RED, WHICH IS A FINDING ABOUT THE
INSTRUMENT AND NOT ABOUT THE UNIT.** `run_wrapper_redtest.sh` perturbs the
shipped wrapper, proves the post-integration harness sees it, reverts, rebuilds
and re-runs the green. On the third re-take that last green read **1552 checked,
9 failed** — the red test's own count — with the source PROVABLY reverted and
the object stamped a second OLDER than the source it was built from. `make`
stat'd the file while the bind mount still showed the pre-revert content. It
only surfaced because the cycle got fast: perturb-build-revert-build now runs in
about three seconds. The wrong artifact is committed at `d3a8253` before the fix
(C12); the repair is the rule `run_harness_stub.sh` already applied one file
over — hash the file from inside the container — plus a `touch` there, because a
hash says the content arrived and says nothing about the mtime `make` compares.

**THE FIVE CALL SITES ARE DEAD FOR TWO DIFFERENT REASONS, AND ONLY ONE OF THEM
IS "NOTHING CALLS IT".** Four sit in the unit-number forms of the `ParseInput`
and `ParseAry` generic interfaces, which ROSCO never selects — it reads its
input file into an array and calls the `_Opt` variants. The fifth is in
`ParseDbAry`, which IS entered 50 times across 24 scenarios; its guard
`IF (CheckName_)` is REACHED 50 times and false 50 times, because
`ReadSetParameters.f90:822` and `:824` both pass `CheckName = .FALSE.` as a
literal. That zero sits between measured non-zeros at 567 and 572 in the same
basic-block chain, which is what makes it a fact about the guard rather than
about instrumentation — the P10 control did not have to be borrowed.

**ALL THREE ARMS ARE REACHED AND COUNTED, AND THE PARTITION CLOSES.** The
reference has one silent path, and the two rules below widened it from two cases
to five:

```
whole unit as a no-op (both error arms)   1547 of 1552
the Words(1)-matches arm alone              61          61 + 1486 = 1547
the neither-matches arm alone             1486
the SILENT arm made to write                 5
                                          ----
arm 1 + arm 2 + arm 3 = 61 + 5 + 1486 =   1552
```

The silent arm is invisible to a no-op *by definition*, so the probe that makes
it WRITE is the only thing that can count it; `1552 − 1547 = 5` would have been
an inference.

**THE ONE RED TEST THAT STAYED GREEN IS NOW RED, AND THAT IS THE WHOLE OF THE
SECOND DISPATCH.** Removing the reference's `CHARACTER(20)` truncation failed
**0 of 1284** and fails **6 of 1552**. Two rules were added to the shared
generator, each mined from something the reference itself declares, and each
one's kill set was measured by running it back out of the corpus:

| rule | what it puts in the corpus | kill set (by ablation) |
|---|---|---|
| **R12_narrowing_width** | 12 cases at the declared width of the reference's own fixed-length CHARACTER locals — every string set from ONE body with a mark at index W, so they agree to W and differ at W+1 | `'20'→'21'`, `'11'→'12'`, `min(len_src,len_dst)→len_src` |
| **R13_staging_capacity** | 256 cases sweeping the staging CAPACITY of a deferred-length CHARACTER output, 0..255 above what the case supplies, every other input held | `'>'→'>='` on the `ErrMsg` cap refusal, killed by **exactly 1** case |

Disjoint kill sets, three mutants and one, neither covering the other's. R12's
raw material is `vit/checks.py::_narrowing_local` transcribed rather than
re-derived (P4) — the static check that already named **this unit** as its
canonical instance, so the two instruments now answer the same question instead
of one answering it alone. R13's finding is the more general one: the staging
capacity is an ARGUMENT of the C contract that **every corpus this generator has
ever produced pinned at one value**, so the refusal arm both implementations
carry was unreachable in every unit, not just this one.

**THREE INSTRUMENT DEFECTS HAD TO BE FIXED BEFORE THE HARNESS EXISTED AT ALL**,
each committed in the repo that owns it (X2): the loop refused
`CHARACTER(*) :: Words(2)` and died in `build_call` (loop `a1d76b0`); VIT's
callee bridge refused `Int2LStr`'s `CHARACTER(11)` result, so the harness failed
to COMPILE inside the translation's own body (vit `f4a711d`); and `harness.sh`'s
bridge-vs-object rule, written for an integrated tree, is **silently wrong** on a
clean one — it dropped the `conv2uc_c` bridge in favour of a stale
`conv2uc.cpp.o`, which links cleanly, reports a number, and runs a DIFFERENT
`Conv2UC` on each side of the comparison. The rule now asks the tree and runs in
both modes.

**A MUTATION SWEEP SCORED 0.000 WITH 31 OF 31 MUTANTS FAILING TO COMPILE**, run
immediately after a `cp` of the translation across the bind mount; the identical
command 90 seconds later compiled 31 of 31. `mutate_guarded.sh` refused to clear
its marker, named the live mutant's hash, and the file was restored before
anything was built. Unit #23's "a `cp` onto a bind-mounted file is read
half-written", at sweep scale.

**AND I EXPLAINED A COUNT INSTEAD OF COMPUTING IT, AGAIN.** Commit `57b2f37`
attributed the wrapper red test's 6 of 1284 to R6 tiling one string body across
every array element; a probe says the corpus separates `Words(1)` and `Words(2)`
in **990 of 1284** cases. The real reason is a source-level fact plus arithmetic
— arm 3 (1230 cases) reports `TRIM(ExpVarName)` and never mentions `Words`, so
it is invisible by construction; of the 54 arm-1/arm-2 cases, `Words(2)` is also
the expected name in 48, leaving 4 + 2 = **6**. Unit #27's rule, walked into
twice now.

The same derivation was re-run at 1552 from the same four counts re-taken there
— 62 / 5 / 61 / 1486 — and gives **9**, which is what the wrapper red test
moves. An arithmetic that survives a 268-case widening of the corpus it is
computed over is a different kind of claim from one that fits once.

**As of 2026-08-13: unit #29 `CheckInputs` is `deferred` and NOT CLOSED** —
`done_check.py` fails `P12`, now at an honest number rather than an
incommensurable one. It is the campaign's largest unit by an order of magnitude
— 857 lines and about 180 validity checks against a previous maximum of three
statements — and **six defects fell out of it, none of them in the
translation**.

Every count below is read from the committed artifact named in its row, not
carried forward in prose.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **1 of 1**, 426 of 426 field rows `IDENTICAL` | VIT declined to construct one (`NON_DISCRIMINATING`); a determinate wrong constant `aviFAIL = -7` scores **0 of 1**, and the **whole unit deleted scores 1 of 1** |
| differential harness (`harness/CheckInputs.json`, `2e2295f`) | **23,076** checked, 0 failed, 0 inadmissible — **this unit's primary evidence** | at 16,769 cases: the `ErrMsg` tail left alone **16,729**; blank-filled **16,769** |
| post-integration harness (wrapper only, `813e7a2`) | **22,824 checked, 0 failed** — one corpus revision BEHIND the row above | not taken |
| gate, 27 scenarios | 5,252,000 values / 351 channels, **0 mismatched** | not taken |
| mutation score (`2e2295f`, all five parts) | **173 behavioural of 192, 78 killed, 0.4509** — below the 1.0 threshold | the reference side is stamped `fortran`, read with `nm` |

**THE KERNEL IS ALIVE AND BLIND, AND THE REASON GENERALISES TO EVERY VALIDATION
ROUTINE.** The wrong-constant stub fails and the whole-unit no-op passes, both
on the same build, so this is blindness and not a broken chain. The cause is in
the capture: the one case KGen could keep is scenario 27's VALID configuration,
so `aviFAIL` is `0` on both sides and `ErrMsg` is never allocated — KGen guards
its comparison on `ALLOCATED` and `errmsg` does not appear among the 426
compared rows at all. The body is not skipped in that case; it runs broadly
(`AWC_Mode=4`, `CC_Mode=1`, `StC_Mode=1`, `PS_Mode=1`, `Fl_Mode=2`,
`IPC_ControlMode=1`) and every check it reaches answers "fine". **A unit whose
only output is an error signal is invisible to any capture taken on a working
configuration.**

**ONE CASE IS NOT A WINDOW THAT WAS TOO NARROW.** The single call site
(`ReadSetParameters.f90:269`, inside `SUBROUTINE SetParameters`) is called once
per scenario at invocation index 1 of its own counter, so all 24 scenarios that
reach it write `CheckInputs.0.0.1` and overwrite one another. Widening
`kgen.invocation` cannot change that; it is a name collision, not an empty
range.

**THREE OF THE 27 SCENARIOS NEVER REACH THE CALL SITE, AND THEY ARE THE
INTERESTING THREE.** Scenarios 10, 14 and 24 set `OL_Mode > 0`;
`ReadControlParameterFileSub` fails on the missing `OL_Mode2_Input.dat` (unit
#17, unit #26) and `SetParameters` RETURNs at line 224 before the call. So
CheckInputs' entire `IF (CntrPar%OL_Mode > 0)` block — the `ALLOCATE`, both
`AddToList` loops and nine checks — is **unreachable in every scenario this
campaign can run**, and the generated corpus is the only thing that tests it.
Unit #26's finding one level up.

**SIX DEFECTS, AND THE UNIT'S SIZE IS WHY.** Four in the corpus generator's R7
rule, two in the callee-declaration generators; details in
`evidence/CheckInputs/harness_scaling_wall.md` and
`evidence/CheckInputs/kernel_callees_header_defect.txt`.

```
generator OOM, exit 137          287,425 cases planned, ~150 KB each, 7.9 GB
                                 -> _KNOB_PAIR_LIMIT = 512, 21,852 cases
a knob naming a WHOLE array      7 of 86; wrote a float into an array slot
a knobbed body vs a knobbed extent   SU_LoadStages_N IS SU_LoadStages' extent
the same block in TWO copies     R7 fixed, R7b still wrong in 3,078 cases
vit_kernel_callees.h             names CFI_cdesc_t and int32_t, includes neither
vit test-validate                DEFINES <callee>_c, declares it nowhere
```

**AND TWO MORE IN THE HARNESS SCRIPT, THE SECOND CREATED BY THE FIRST REPAIR.**
`checkinputs_callees.f90` defines `addtolist_c` and so does the integrated
`addtolist.cpp.o`. Dropping the object to keep the bridge fixed the duplicate
symbol and produced a LOOP — on an integrated tree the Fortran `AddToList` is a
wrapper around `addtolist_c`, so bridge → wrapper → bridge, SIGSEGV on case 0
with no message and a core file. Inverted: drop the bridge, keep the object.
Then `vit_mutate` refused to score (`baseline is not green (nocompile)`) because
`<stem>.cpp.o` is dropped and the integrated wrapper calls `checkinputs_c`; the
integration shim now goes into the Makefile rather than onto post mode's command
line.

**THE MARGIN IS ONE `memset`, AND TWO RED RUNS DID NOT SETTLE ITS DIRECTION.**
`ErrVar%ErrMsg` is `CHARACTER(:), ALLOCATABLE`, so an assignment reallocates to
exactly `LEN` and the reference has no bytes past the new length. Leaving the
previous message's tail fails **16,729 of 16,769**; blank-filling fails
**16,769 of 16,769**; clearing to NUL passes. The two red counts differ by 40
and both name `ErrVar.ErrMsg`. What settled it was the first differing BYTE —
`a=0x20 b=0x00` at exactly index `n_ErrMsg` — because the oracle side is a
zeroed buffer the bridge writes `n_ErrMsg` bytes into.

**THE MUTATION RUN MEASURED THE MUTANT AGAINST ITSELF, AND ONLY THE SHAPE OF
THE NUMBER SAYS SO.** 173 mutants, 4 killed, **0.0231**. On an integrated tree
`vit_mutate`'s build routes the harness's Fortran side through the wrapper, into
`checkinputs_c`, into `vit_integration_shim.o`, into the harness's own compiled
copy — the mutant. Both sides run the mutant and every behavioural difference
cancels. **169 survivors on a corpus that passes 16,769 of 16,769 against real
Fortran is the tell**: 40 `compare_op` flips cannot all be equivalent. It is
unit #21's and unit #24's "a green that measured nothing" with the sign flipped,
and nothing in `mutation/<U>.json` records which side the reference was — the
harness artifact has a `measures:` field for exactly this and the mutation
artifact has none.

The measurement needs the CLEAN tree, where `ReadSetParameters.f90.o` carries
the real 857-line body — the configuration `harness/CheckInputs.json` was taken
in. **Not re-run in this dispatch**, and the reason is stated rather than
hidden: reset, rebuild, regenerate, and a ~50-minute sweep, after six tool
defects had already consumed the budget. Kept as
`evidence/CheckInputs/mutation.integrated-build-INVALID.json` so the number
cannot be read as a score. **The unit is integrated, gate-green and
harness-green; it is not closed.**

**THREE PINS IN `harness/ranges.toml`, ALL BECAUSE THE ORACLE READS OUT OF
BOUNDS.** `CheckInputs` validates `AWC_NumModes` against 0 and against 2 and
never against `SIZE(CntrPar%AWC_freq)`, and then loops over it. Case 9544 killed
the Fortran with `AWC_NumModes = 99999` against `n_AWC_freq = 28`. The
translation survived the same loop, which is luck: ~800 KB past a 28-element
allocation happened to be mapped on the C++ heap and not on the Fortran one.
Bounds are the reference's own predicates (`-1..3`, `0..2`, `0..2`), so no
branch is lost; the cost — a count larger than its array is untested — is in the
file. Seventh of the "the reference has no answer" family.

**As of 2026-08-13: unit #28 `wrap_360` is `integrated` and CLOSED**, first
dispatch. Five layers available, five run, all green, all red-tested. **It is
unit #27's sibling and its counterexample**: three statements, one screen down
in the same file, the opposite comparison pair — and where `wrap_180`'s two arms
are dead at all six of its call sites, `wrap_360` has **one live arm and one
dead one**, and every layer says which.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **41 of 41**, 41 of 41 field rows `IDENTICAL` | VIT's own (`x × 1.00001`); wrong constant `-7.25` **0 of 41**; both arms deleted **20 of 41**; the low arm alone **41 of 41** |
| differential harness vs clean Fortran | **134** checked, 0 failed, 0 inadmissible | no-op **127**; both arms **51**; low arm alone **36**; high arm alone **15**; the sibling's comparison spelling **7** |
| mutation score | **11 of 11 killed, 1.000**, **0 declared equivalent**, 0 no-compile, 5 operators | the score *is* the red test, eleven times |
| post-integration harness (wrapper only) | **134 checked, 0 failed** | the wrapper hands `-x` to `wrap_360_c`: **129 of 134**, revert verified green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | whole unit → `0.0` moves **84,477**; both arms deleted **31,579**; high arm alone **31,579**; low arm alone **0** |

**THE SAME PARTITION FROM THREE INSTRUMENTS, AND THEY DISAGREE ABOUT ONE ARM.**
The two arms are disjoint, so deleting them singly must account for deleting
both — and all three layers say so independently:

| instrument | low arm alone | high arm alone | both |
|---|---|---|---|
| kernel, 41 cases | 0 | 21 | 21 |
| differential harness, 134 cases | **36** | 15 | 51 |
| gate, 5,252,000 values | 0 | 31,579 | 31,579 |

The low arm (`x < 0 → x + 360`) has **0 hits at both call sites in all 27
scenarios**, so the two bit-exact layers move nothing when it is deleted, and
the generated corpus is the only thing that tests it. Both zeros have their
control on the **same build** — the whole-unit no-op and the both-arms stub move
84,477 and 31,579 — which is what makes them blindness rather than a broken
chain. Unlike unit #27's dead arms this is not caused by a defect in the
simulation harness: `360*Time*AWC_freq(1)` is non-negative by construction.

**THE CALL SITE WAS CHOSEN ON THE ARM, AND THE HOT SITE HAPPENED TO BE THE RIGHT
ONE.** `Controllers.f90:515` runs 3,999 times in scenario 2 and **every one of
those calls takes the pass-through arm** — a kernel captured there would be unit
#27's situation repeated. `Controllers.f90:845` runs 15,999 times in scenario 22
and 15,199 of them wrap. Unit #24's rule is served by its purpose rather than by
its literal procedure: the committed coverage settles it one step earlier and
cheaper (unit #25), because line 473 has hits in exactly one scenario and it is
not 515's. Said out loud, as unit #25 requires.

**THE INVOCATION WINDOW'S THIRD RANGE WAS PREDICTED EMPTY BEFORE EXTRACTING.**
Unit #25's arithmetic, which costs one query: the window is
`0:0:1-20, 0:0:12000-12020, 0:0:23900-23920` and the site has 15,999 calls, so
the last range contributes 0 and the capture is 20 + 21 = **41 cases**. It came
back 41, with exactly those indices.

**THE DEFECT THIS UNIT IS ACTUALLY EXPOSED TO IS ITS SIBLING.** `wrap_180` is
`.le.` low and `.gt.` high — the half-open interval `(-180, 180]`. `wrap_360` is
`.lt.` low and `.ge.` high — `[0, 360)`. Reading one across into the other moves
exactly `x = 0.0`, `x = -0.0` and `x = 360.0`, and **nothing else in the whole
real line**. That stub fails 7 of 134 differential cases, and the corpus rules
that give it those 7 are R6's predicate knob and the signed-zero rung unit #14
added — the same rung that was unit #24's entire margin.

**A PROBE COMPARING AGAINST `2·π` REPORTED FAILURE ON A CORRECT RUN.** The
kernel's caller multiplies by ROSCO's own `D2R = 0.01745329251`
(`Constants.f90:23`, eleven digits), so the per-case difference a wrap deletion
produces is `360·D2R = 6.2831853036`, not `2·π = 6.2831853072` — a disagreement
in the tenth digit, above the printed precision of the kernel's own difference
line. **P7 reaches the probe as well as the translation.** The same script's
first version recovered the input domain from a *model* of the call site rather
than from a measurement, and that model reproduced only 6 of 41 captured values
bit for bit; it was replaced by three stub runs that had to happen anyway.

**As of 2026-08-13: unit #27 `wrap_180` is `integrated` and CLOSED**, first
dispatch. Five layers available, five run, all green, all red-tested — and the
thing to know about this unit is not the translation, which is three statements,
but that **the two bit-exact layers watch it run 675,987 times and never once see
it do the thing it exists to do.**

| layer | result | red-tested |
|---|---|---|
| kernel replay | 62 of 62, **13,950 of 13,950** field rows `IDENTICAL` | VIT's own (`x × 1.00001`); plus a determinate wrong constant `-7.25` at **0 of 62** and the branch-deleting stub at **62 of 62** |
| differential harness vs clean Fortran | **136** checked, 0 failed, 0 inadmissible — **this unit's primary evidence** | no-op **130 of 136**; both branches deleted **31**; low branch alone **13**; high branch alone **18** |
| mutation score | **11 of 11 killed, 1.000**, **0 declared equivalent**, 0 no-compile, 5 operators | the score *is* the red test, eleven times |
| post-integration harness (wrapper only) | **136 checked, 0 failed** | the wrapper hands `-x` to `wrap_180_c`: **130 of 136**, revert verified green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | whole unit as a no-op moves **206,976**; **both branches deleted moves 0** |

**BOTH WRAPPING BRANCHES ARE DEAD AT ALL SIX CALL SITES IN ALL 27 SCENARIOS, AND
THE UNIT IS NOT DEAD.** This is a different shape from units #1, #21 and #26,
which had no live call site at all. Here the unit is called 675,987 times across
23 of 27 scenarios and **every one of those calls takes the pass-through arm**.
Three independently derived counts say so and they are equal:
hits at the FUNCTION line = hits summed over the six CALL SITES = hits on the
ELSE line = 675,987, with 0 on each branch body. The P10 control is in the same
file: `wrap_360`, two screens down, is the same shape with a live branch
(`x >= 360`, 15,199 hits, scenario 22), so a branch body does get its own
coverage entry when it runs.

**THE REASON IS A DEFECT IN THE SIMULATION HARNESS, AND READING THE SCENARIO
SOURCE GIVES THE OPPOSITE ANSWER.** Scenarios 7 and 27 inject
`avrSWAP[36] = 350°` specifically to drive this unit past +180, under the comment
*"Inject avrSWAP values not handled by call_controller"* — and
`rosco/toolbox/control_interface.py:211` overwrites it from
`turbine_state['Yaw_fromNorth']` before `call_discon`. Two of the six injected
indices *are* handled by `call_controller`; index 23 survives only by coincidence
(`Y_MeasErr` carries the same value) and index 36 is replaced by the accumulated
yaw position, which starts at 0. Refuted from the committed `.dbg` without
running anything: a heading of 350 would force `Yaw_Err` into `[-3.3, +34.0]`, and
**10,632 of 23,999 timesteps sit outside it**. NOT FIXED — repairing it changes
what the 27 scenarios feed the controller, which moves every baseline and every
committed compared count (X3). Logged under Open.

**THE GATE'S OWN RED TEST IS ITS CONTROL, SO NO BORROWED CONTROL WAS NEEDED.**
Two perturbations on the same libdiscon, both anchored to
`double wrap_180(double x) {` (asserted unique in the tree, unit #26): the
whole-unit no-op moves 206,976 of 5,252,000, and the stub with both branches
deleted moves 0. The first proves the chain is alive, so the second is blindness
rather than breakage. `unwrap` had to re-run GetWords' perturbation for this;
here the unit supplies its own.

**THREE INSTRUMENTS, ONE STUB, THREE ANSWERS.** Both wrapping branches deleted:
gate **0 of 5,252,000**, kernel **62 of 62 PASSED**, harness **31 of 136
FAILED**. Two bit-exact layers over 675,987 real calls certify a translation
missing half its body; a 136-case generated corpus does not.

**THE CALL SITE WAS CHOSEN ON ARGUMENT DOMAIN, AND THE COVERAGE SETTLED IT MORE
CHEAPLY THAN A STUB WOULD.** Three of the six sites pass `atan2(…)·R2D`, whose
range is `[-180, 180]`, so at those the `x > 180` branch is unreachable **by
construction** and not merely by corpus. `Controllers.f90:400` is a plain sum, so
it admits both branches in principle. Unit #24 says to run the branch-deleting
stub at each candidate site before spending a cycle on one; here the committed
coverage answers it one step earlier (unit #25) because the branch bodies have
zero hits at *every* site — no stub could separate sites on that axis. Purpose
served, literal procedure not followed, said out loud as unit #25 requires.

**THE MUTATION SCORE'S MARGIN IS TWO CASES, AND THEY WERE COUNTED RATHER THAN
TRUSTED.** `'<=' → '<'` and `'>' → '>='` each differ from the reference on
**exactly one input value**, so `2 of 136` is not a sample statistic — it is the
multiplicity of `-180.0` and `+180.0` in the case file, which R6 puts there twice
each (the literal ladder and the predicate knob). Computed both ways from the
`.bin` the harness ran. Take that block away and both mutants survive at any
corpus size. **Nothing survived and nothing was declared equivalent**, so this
unit has no `.equivalences.json` and no `.undeclared.json`, and the absence is
structural rather than an omission.

**TWO RED TESTS REPORTING `130 of 136` ARE NOT ONE MEASUREMENT, AND I GOT THIS
WRONG FIRST.** The commit message of `ad9f755` explained the matching counts from
an argument — *"the same reason: 0.0 and -0.0 map to themselves under negation"* —
and the wrong claim is left standing in the git log (C12) beside the artifact that
corrects it. The blind sets are **different six cases, overlapping in two**. The
no-op is blind where `ref(x)` *is* `0.0`; the sign flip is blind on the **four
boundary cases**, because `.le.` on the low guard and `.gt.` on the high one send
*both* endpoints to `+180` — the very asymmetry the translation exists to preserve
is what hides that perturbation. And `-0.0` is in **neither** set: the one
mechanism the wrong claim named is the one the corpus rules already close. Unit
#26's census compares red-test counts *across* corpora; this is the same hazard
with the corpus held fixed, where nothing looks wrong at all. Logged under Open
as a candidate check.

**As of 2026-08-12: unit #26 `unwrap` is `integrated` and CLOSED**, on its second
dispatch — the first produced everything except the integration half and ended
before committing it; the driver committed that work at `284df58` and
re-dispatched, because `Functions.f90` is protected and only a session holds
`integration_only` standing. **THREE OF THE FIVE LAYERS ARE UNAVAILABLE OR BLIND
HERE, AND ALL THREE ABSENCES ARE MEASURED RATHER THAN ASSERTED.**

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT AVAILABLE** — no scenario reaches either call site, so there is no state to capture | n/a; the deadness itself is the measurement (`coverage_deadness.py`, exit 0) |
| differential harness vs clean Fortran | **403** checked, 0 failed, 0 inadmissible — **this unit's primary evidence** | the unit as a no-op fails **373 of 403**. At the narrower 377-case corpus: no-op **363**, `+2·PI` loop deleted **4**, `-2·PI` loop deleted **361** |
| mutation score | **37 of 40 killed, 1.000**, 3 declared equivalent, 0 no-compile, 7 operators | undeclared runs committed at **0.925** and at **0.875** against the pre-widening corpus |
| post-integration harness (wrapper only) | **403 checked, 0 failed** | the `--reverse-copy` line deleted from the wrapper fails **370 of 403**, naming `ErrVar.n_ErrMsg` and `ErrVar.ErrMsg` and nothing else |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched — **and it establishes nothing about this translation** | the whole unit as a no-op moves **0 of 5,252,000**. Same-build control: **1,857,893** |

**THE UNIT IS DEAD AT BOTH CALL SITES IN ALL 27 SCENARIOS, AND THE TWO SITES ARE
DEAD FOR TWO DIFFERENT REASONS.** Third such unit after #1 `AddToList` and #21
`UpdateZeroMQ`, and the first where one site is not simply an unentered guard.
`Controllers.f90:339` is behind `OL_Mode > 0 .AND. Ind_GenTq > 0`, never true in
407,976 evaluations. `ReadSetParameters.f90:807` is not that shape: scenarios
**10, 14 and 24 do configure `OL_Mode = 2` and do reach the block**, then
`Read_OL_Input` fails on `Examples/example_inputs/OL_Mode2_Input.dat`, absent
from this tree (unit #17 measured the same absence from the other side), and the
`RETURN` two statements later takes them out. Reading the guard alone would have
said *no scenario configures it*, which is false. Print the whole path from the
guard to the call.

**BOTH POST-INTEGRATION NUMBERS WERE PREDICTED BY ARTIFACTS THAT ALREADY
EXISTED.** The gate red test's 0 by `coverage_deadness.py`, from committed
coverage, before any of it ran. The reverse-copy red test's **370** by
`errmsg_extremes_probe.txt`, which was written to answer a *mutation* question
about an unreachable capacity guard and counts exactly 370 `assign_errmsg` calls
over the same 403-case corpus. Two instruments meeting at one number is unit
#24's cross-check, and here it also rules out the bind-mount clock skew `make`
warned about on every run: a stale library returns the green, not a figure that
matches an independent count.

**A RED TEST TAKEN AT A NARROWER CORPUS IS NOT A RED TEST FOR THE WIDER ONE.**
`evidence/unwrap/README.md` tabulated three stub red tests as "of 403"; all three
were taken at **377** and their committed JSON says so. Only the green was
re-taken when the corpus was widened. Re-run at 403 the no-op fails **373**, not
363; the other two were not re-run and their cells now say `?` rather than
repeating the 377 figures. Corrected in the README, and raised in DECISIONS.md as
a **candidate method amendment**: a red test certifies a green only if their
`checked` counts match, which every artifact this campaign writes already records
and nothing checks.

**The gate artifact committed at `aabf439` had to be thrown away.** It predates
`vit integrate`, so it measured a libdiscon containing no `unwrap` C++ at all —
and it read 5,252,000 / 0, the identical number the integrated build produces.
Unit #23's re-take rule, one build earlier: a gate green that passes either way
must be re-taken, and the tell is never the number.

Unit #25 `sigma`'s section is directly below; unit #24 `saturate` is below that.
`scripts/done_check.py sigma` returned COMPLETE, 13 of 13
(`evidence/sigma/done_check.txt`, both takes kept).

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 27, clean `Controllers.f90:526` | **14,136 field rows, all IDENTICAL**, 62 of 62 | a constant stub reading no argument passes **0 of 62**; **BOTH CLAMPS PASS 62 of 62** |
| differential harness vs clean Fortran | **1,069** checked, 0 failed, 0 inadmissible | no-op **1069 of 1069**; lower clamp deleted **116**; upper clamp deleted **94**; the ramp deleted **812** |
| mutation score | **42 of 42 behavioural killed, 1.000**, 3 declared equivalent, 0 no-compile | the undeclared run is committed at **0.933** so the survivors are on the record before they were excused |
| post-integration harness (wrapper only) | **1,069 checked, 0 failed** | the `--reverse-copy` line deleted from the wrapper fails **1,022 of 1,069** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | the cubic replaced by `y0` moves **229,165 of 5,252,000**, 18 channels, 3 scenarios; revert verified |

**THE KERNEL'S RED TEST ON BOTH CLAMPS IS NOT SMALL, IT IS ZERO.** `sigma` is
`y0` below `x0`, `y1` above `x1`, and a Hermite cubic between. Deleting either
clamp from the shipped translation passes the kernel **62 of 62** and fails the
differential harness **116 of 1069** and **94 of 1069**. The cause is a property
of the shipped program: `IPC_Vramp` is `9.120  11.400` in **all 14**
`Examples/DISCON*.IN`, so at this call site the two bounds are constants, and
the 62 captured wind-speed estimates run **9.5804 .. 10.7998** — strictly
inside them. What reaches the clamps in the harness is R6's two-sided-predicate
block, added at unit #20: `x` against `x0` and `x` against `x1` are predicates
between two VARIED quantities, so 20 cases set one side from the other at
equality and at its two neighbouring representable values. A rule added five
units ago is the entire reason two of this unit's three branches are tested at
all.

**THE SECOND CALL SITE WAS REJECTED ON A COUNT, NOT ON A STUB, AND THAT IS A
DEVIATION RECORDED RATHER THAN HIDDEN.** Unit #24's rule is to run the
branch-deleting stub at each candidate site. The alternative,
`ControllerBlocks.f90:612`, executes **800 times** while `kgen.invocation` is
`0:0:1-20,0:0:12000-12020,0:0:23900-23920` — two of the three ranges are past
its last call, so a capture there is at most 20 cases, all in the first 20
timesteps of a load ramp. Coverage also gives line 502 (`sigma = y0`) no
scenario-9 hits, so it cannot reach the lower clamp either. See DECISIONS.md.

**THE LIVENESS STUB IS `-7.25`, NOT ZERO, AND THE CALL SITE DECIDED THAT.** `y0`
is the literal `0.0_DbKi` at `Controllers.f90:526`, so a zero stub agrees with
every case that takes the lower clamp and would have measured the argument list
rather than the kernel. Unit #22's determinate-finite-and-wrong rule, with this
call site's own arguments deciding what "wrong" is.

**`--reverse-copy` WAS DECIDED BEFORE INTEGRATING, AND ITS RED TEST REPRODUCED A
NUMBER A SECOND INSTRUMENT HAD ALREADY PRODUCED.** Unit #23's two greps name
`ErrVar%ErrMsg` as the one scalar field of an `INTENT(INOUT)` view-type dummy
this unit writes. Deleting the generated `CALL vit_copy_scalars_to_errorvariables`
fails **1,022 of 1,069** — and `evidence/sigma/errmsg_extremes_probe.txt`,
written to answer a *mutation* question, independently counts **1,022**
`assign_errmsg` calls over the same corpus.

**THREE SURVIVORS, ALL THE `CHARACTER(:), ALLOCATABLE` IDIOM, AND NONE IN THE
ARITHMETIC.** The cubic, both clamps, all four coefficients and every literal in
them die — including the two `negate_cond` mutants on the clamps the kernel
cannot see (991 and 914 of 1069). `532e4d37` is EQUIVALENT and proved over all
**4,294,967,296** values of a 32-bit int; `2524b715` and `7ad82e7d` are
UNREACHABLE OVER THIS CORPUS, which is a blind spot and is recorded as one. The
capacity guard's unreachability is **weaker here than in the four earlier units
carrying the same site** — their messages are fixed-length literals, this one's
grows with its input — and the declaration says so.

**`**` WITH AN INTEGER EXPONENT WAS MEASURED, NOT READ.** 60,022 bit patterns
through `v**2` against `v*v` and `v**3` against `v*v*v`, both signed zeros
included: **0 differ** (`evidence/sigma/int_pow_probe.txt`). `(x0-x1)**3` is
written once in the translation — the reference spells it four times and it is
the same value in all four, so three of those are restatements.

---

**As of 2026-08-12: unit #24 `saturate` is `integrated`**, on its second
dispatch. The first closed `blocked` on a block of a kind this campaign had not
had before — **the mutation score was not low, it was ABSENT**, `cppmutate`
generating ZERO mutants for a translation whose body is a call expression, which
`done.py` fails by name as `mutation_no_mutants`. The second dispatch closed it
by adding the operator and paying its campaign-wide price in the same cycle. Its
section is directly below; unit #23 `interp1d` is below that.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 13,950 field rows, **all IDENTICAL** | zero stub passes only **21 of 62**, moving 41 rows; **the MIN deleted passes 62 of 62** |
| differential harness vs clean Fortran | **451** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **315 of 451**; no saturation at all fails **202 of 451** |
| mutation score | **1.000 — 4 of 4 behavioural killed**, 2 declared equivalent and proved, 0 no-compile | the undeclared run is committed at **0.667** so the survivors are on the record before they were excused |
| post-integration harness (wrapper only) | **451 checked, 0 failed** | the two BOUNDS swapped at the CALL, rebuilt between edit and run: **271 of 451** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | the saturation removed moves **2,255,249 of 5,252,000** — the largest red in this campaign, matching no other |

**THE OPERATOR SET NOW REACHES A CALL EXPRESSION, AND THE X3 COST WAS MEASURED
RATHER THAN ARGUED.** Three operators added to `harness/cppmutate.py` (loop
`b9fb5ee`): `drop_call`, `swap_call_args`, `swap_callee`. Unit #22 had refused
this class because *a new operator can produce a SURVIVOR in a unit that closed
at 1.000*. That is two questions and only one is expensive:

* **Does it invalidate what is already scored?** No, and it cannot — `_mid` is
  content-derived, so an addition cannot move an existing id. Across all 24
  scored units: **0 ids lost, 35 gained, 7 units affected.** A re-take is what
  LOST ids would force; zero lost is a debt, and the debt is only the new
  mutants in the affected units.
* **Does it produce survivors?** **Yes — 8, in 3 units that had closed at
  1.000.** GetPath **0.500**, GetRoot **0.333**, GetWords **0.667**, every one
  the same `std::min(len_src, len_dst)` bounded-copy clamp or its argument swap.

The debt was paid here, into separate artifacts: `mutation/<U>.call_operators.json`
for the 6 other affected units, with `mutation/<U>.json` untouched because it is
not wrong — it says what the nine operators found and it still does. Sweep and
transcript: `evidence/saturate/call_operator_retake.{sh,txt}`. **`ReadAvrSWAP` is
the one affected unit NOT measured**: its generated `readavrswap_test.cpp`
predates its translation's current signature, so the baseline will not compile
and `vit_mutate.py` refused before any mutant ran
(`evidence/saturate/call_operator_retake.ReadAvrSWAP.txt`).

**TWO RESTRICTIONS, BOTH FORCED BY A LIVE SWEEP.** A token-level operator that
rewrites a call is type-blind: unrestricted, the string-handling units came back
38%, 43%, 55%, 60%, 73%, 80%, 89%, 91% and 100% **unbuildable**, and
`NOCOMPILE_LIMIT` refused all of them. Raising that limit would have been the
wrong repair — it is what catches a genuinely broken build. Restricting
`drop_call` and `swap_call_args` to `_VALUE_PRESERVING` took 231 mutants across
13 units to 35 across 7, all of which compile. And `_call_sites` needed a real
test for a definition head, because a constructor's member-initialiser list is
followed by `:` rather than `{`: a **parameter is two bare identifiers in a
row**, a shape no C++ expression has.

The hand measurement the first dispatch made keeps its value as a **cross-check
rather than a substitute**: `evidence/saturate/hand_mutants.txt` reports 151 and
100 for the two clamp deletions, and the tool's two `drop_call` mutants report
the same 151 and 100. Two instruments, two numbers, taken a dispatch apart.

**THE WHOLE UNIT IS TWO INTRINSICS, AND THE SPELLING WAS MEASURED.** gfortran's
`MAX`/`MIN` are `fmax`/`fmin` bit-for-bit; **both** branch spellings are wrong at
a signed zero and at a NaN, in opposite directions. Over 12,167 triples the
shipped spelling differs from gfortran on **0**, branch A on **789**, branch B on
**561** (`evidence/saturate/saturate_expr_sweep.*`). This is not unit #14's rule
inverted — there the Fortran wrote an explicit `IF (CornerFreq < 0)` and `fmax`
was the *mutant's* answer; here the Fortran writes the intrinsic. The sharp part
is what says so: **the branch spelling dies on exactly 1 of 451 corpus cases**,
the negative-zero one, which exists only because unit #14 added a signed-zero
block after its own `dict.fromkeys` dedup absorbed `-0.0` into `0.0`. A corpus
addition made two units ago is the entire margin by which this unit's only real
defect class is visible.

**THE CALL SITE WAS CHOSEN ON A STUB, NOT ON A HIT COUNT.** The kernel is alive
(zero stub 21 of 62) and blind to the upper clamp: deleting the `MIN` scores
**62 of 62 PASSED, 14,260 of 14,260 IDENTICAL**. The site tried first,
`ControllerBlocks.f90:332` — 407,976 hits, 23 scenarios, non-aliased, everything
coverage recommends — is *worse*: there the passthrough stub passes **62 of 62**
and **neither** clamp is visible, because `BlPitchCMeas` is interior to its bounds
in every captured case. Both artifacts are committed. "The kernel is alive" and
"the kernel can see what the unit is FOR" are two claims, and only the second
chooses a call site.

**TWO KGEN DEFECTS, THE FIRST CALL SITE IN THIS CAMPAIGN INSIDE A FUNCTION BODY.**
`gen_kernel_callsite_file` sets `tosubr` to convert the hoisted parent block into
a SUBROUTINE; `SubProgramStatement.tokgen` read **`tosurb`**, a transposed typo,
in both copies of the file. So the kernel carried a `FUNCTION` header, an
`END SUBROUTINE`, and a driver that `CALL`s it — 29 compile errors. Fixed in KGen
(X2, `d3d6516`); inert for a SUBROUTINE parent block by construction, and the
subroutine path was exercised end to end twice at 62/62. The fix does **not** make
such a kernel work: the parent block becomes `SUBROUTINE picontroller` and the
callsite statement `PIController = saturate(...)` is then an assignment to the
subroutine's own name, and the function RESULT is neither declared nor captured as
state. That second gap is recorded rather than attempted, and it blocked nothing
here — 8 of `saturate`'s 17 call sites are in SUBROUTINE scope.

---

**As of 2026-08-12: unit #23 `interp1d` is `integrated` and CLOSED**, first
dispatch. **Five layers, all five alive — and one of them went RED on a defect
the other four passed.** Its section is directly below; unit #22 `identity` is
below that.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 248 field rows, **all IDENTICAL** | determinate zero stub **0 of 62**, `a_op` moves in all 62; **the no-boundary-branches stub PASSES 62 of 62** |
| differential harness vs clean Fortran | **497** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **497 of 497** |
| mutation score | **66 of 66 behavioural killed, 1.000**, **5 declared equivalent**, 1 no-compile | the inherited corpus scores **0.865, thirteen survivors** — 8 closed, 5 declared |
| post-integration harness (wrapper only) | **497 checked, 0 failed** — after a fix | **IT FAILED FIRST: 454 of 497**, see below; the deliberate red re-take reproduces the same 454 |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `interp1d_result` × 1.000001 moves **1,341,803 of 5,252,000**, revert verified |

**THE WRAPPER THREW AWAY EVERY WRITE THIS UNIT MAKES, AND THE KERNEL AND THE
GATE BOTH PASSED IT (C12).** `vit integrate --apply` without `--reverse-copy`
populates `ErrVar_view`, calls the C++ and returns — nothing copies the view
back — so `interp1d`'s writes to `ErrVar%aviFAIL` and `ErrVar%ErrMsg` went
nowhere. The kernel marshals the view itself and never touches the wrapper; the
gate's 27 scenarios hand this unit well-formed monotone tables with
`aviFAIL == 0`, so the dropped fields are never written. **The gate is not blind
to the unit** — a perturbed return value moves 1.3 million values — **it is
blind to an entire output of it.** The post-integration harness caught it on the
first run, 454 of 497, naming `ErrVar.ErrMsg` and `ErrVar.n_ErrMsg` and nothing
else. The failing artifact is committed
(`evidence/interp1d/harness.postintegration.NO-REVERSE-COPY-FAILS-454-of-497.json`).
DECISIONS.md carries the candidate: `--reverse-copy` is derivable from the
unit's own body and should be inferred or REFUSED, not left to the caller.

**THE KERNEL CANNOT SEE TWO OF THE THREE INTERPOLATION BRANCHES.** A stub with
`IF (xq <= MINVAL(xData))` and `ELSEIF (xq >= MAXVAL(xData))` deleted scores
**62 of 62 PASSED, 248 of 248 IDENTICAL**: every query captured at the wind-speed
estimator's call site is interior, at any invocation window. Both error branches
and the whole `IF (ErrVar%aviFAIL < 0)` tail are unentered too — `avifail` is 0
in all 62 cases. So the differential harness is the only instrument on five of
this unit's paths, and on two of them **it was blind as well**: `xq` against
`MINVAL(xData)` is a predicate whose other side is a REDUCTION, with no name to
pin and no value in the case stream, and `<=`-vs-`<` survived 473 cases. New
rule **R10** draws the array body first and sets the scalar FROM the reduction,
over the body and its reverse. 0.865 → 0.920 → 0.930 → **1.000**.

**THE REFERENCE ABORTS ON ITS OWN ERROR PATH.** `SIZE(xData) /= SIZE(yData)`
reallocates `ErrVar%ErrMsg` to 38 characters and then formats 53 into that
record — `Fortran runtime error: End of record`, exit 2, measured. The fifth
upstream ROSCO defect here, and the second of the "the reference does not
return" family after unit #17. `harness/ranges.toml` grew a **third kind of
entry** for it, `same_as`, which says two INPUTS are only jointly admissible —
a range narrows one parameter alone, `no_oracle` excludes an output, and neither
can say this.

---

**As of 2026-08-12: unit #22 `identity` is `integrated` and CLOSED**, first
dispatch. **Five layers, all five alive — and the kernel's VERDICT LINE is not
one of them.** Its section is directly below; unit #21 `UpdateZeroMQ` is below
that.

| layer | result | red-tested |
|---|---|---|
| kernel replay | 62 cases, 225 fields, **13,950 of 13,950 IDENTICAL** | **the verdict cannot go red** — a no-op stub scores `✓ 62/62 passed`; the DETERMINATE wrong-constant stub scores **0 of 62** and is what says the comparison is alive |
| differential harness vs clean Fortran | **29** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **28 of 29**; the survivor is `n == 0`, where neither side writes |
| mutation score | **20 of 20 behavioural killed, 1.000**, 0 declared equivalent, 0 no-compile, on the first run | 9 of the 20 are CRASHES, so killed-by-comparison is **11 of 20** |
| post-integration harness (wrapper only) | **29 checked, 0 failed** | `CALL identity_c(n, A)` → `identity_c(n - 1, A)`, rebuilt between edit and run: **28 of 29** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | the diagonal written `2.0` moves **1,462,798 of 5,252,000** across **115 channels and 22 scenarios** |

**A NO-OP STUB PASSES THE KERNEL BECAUSE `NaN > 1.D-14` IS FALSE.** KGen scores
an array field `IN_TOL` when `SQRT(SUM((var-ref)**2)/n)` is not greater than the
tolerance, and a NaN is not greater than anything — so a translation that writes
nothing leaves the automatic array uninitialised, `LocalVar%WE%P` comes back NaN,
the field scores `IN_TOL`, and the kernel prints `Number of verification-passed
cases : 62`. **This is not the tolerance-versus-magnitude shape unit #19
recorded**, and the difference was measured rather than argued: a determinate,
finite wrong-constant stub — the 3x3 identity with `2.0` on the diagonal — scores
0 of 62 with `p` OUT_TOL at `rms = 0.378`. `p` is of order 1 and the comparison
is alive. C12: the wrong artifact is committed
(`evidence/identity/kernel.noop-stub.verify_fields.csv`) and the instrument is
NOT fixed here — a NaN guard changes the pass basis of every kernel this
campaign has run (X3, SPEC §8.4). DECISIONS.md carries it as a candidate.

**THE KERNEL CANNOT CONSTRAIN THE ARGUMENT AT ALL.** The reference's one call
site is the literal `identity(3)`, so every captured case has `n == 3` and no
invocation window can widen it — `n` is not data. A constant stub reading no
argument and writing a hardcoded 3x3 identity scores **13,950 of 13,950
IDENTICAL**. Everything this campaign knows about `identity` as a function of
`n` comes from 29 differential cases at five values of it.

**`bridge_feasible: unknown` WAS ANSWERED DIFFERENTLY BY THE TWO GENERATORS, AND
ONE ANSWER WAS A DEFECT.** `vit interface` has crossed an array-valued FUNCTION
RESULT since the campaign began — `REAL(C_DOUBLE), INTENT(OUT) ::
identity_result(*)`, with the wrapper declaring `REAL(8), DIMENSION(n, n) :: A`.
`test_validate.generate_fortran_bridge` declared the same result a SCALAR:
*Incompatible ranks 0 and 2 in assignment*. Third disagreement between these two
generators (units #8, #17, #22), second on a RESULT. Fixed in VIT `ab75fa0`
(X2). The loop had the matching gap and it was worse than a build failure — the
result mapped as a scalar INPUT and **the unit's only output varied on the
±1e3 default** — fixed in `20b0dbb` together with two defects that fell out of
it: a predicate knob on an EXTENT desynchronised the case stream, and **R5
emitted ONE SHAPE for every unit with ONE free extent**, silently, while
reporting "1 varied extent(s) at [3]".

**TWO THINGS ARE UNFALSIFIABLE HERE RATHER THAN UNTESTED.** Transposing the
index expression moves 0 of 29 cases at every `n`, because the identity matrix
is symmetric — VIT's own column-major rule is enforced on this translation and
cannot be tested by it. And the STRIDE has no mutant at all: `cppmutate`'s
`_OPERAND` is `identifier | number`, so the parenthesised `(j - 1) * n` produces
no `arith_op`, `drop_factor` or `swap_operands` mutant — and the parenthesis is
required by VIT's own `exponent-grouping` check. Run by hand instead
(`evidence/identity/stride_probes.txt`): `* 3` fails 2 of 29 and `/ n` fails 2 of
29, and **one of those two cases is the shape the R5 fix adds**.

## Unit #21 — UpdateZeroMQ — 2026-08-12

**Unit #21 `UpdateZeroMQ` is `integrated` and CLOSED**, first
dispatch. **Four layers alive, one absent by measurement** — the kernel, and it
is the first unit in this campaign with no kernel at all: both call sites are
guarded by `ZMQ_Mode > 0`, which is 0 in all 14 inputs, so there is no state to
capture. Its section is directly below; unit #20 `StateMachine` is below that.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT AVAILABLE** — clean `DISCON.F90:103` (the guard) 407,976 hits, `:104` (the CALL) **0**; `:141` 36,024, `:142` **0** | n/a |
| differential harness vs clean Fortran | **8,334** checked, 0 failed, 0 inadmissible — **6 outputs declared `no_oracle`** | the unit as a no-op fails **4,167 of 8,334**, every case that reaches the body |
| mutation score | **12 of 12 behavioural killed, 1.000**, **8 declared equivalent**, 0 no-compile | the inherited corpus scores **0.2593, forty survivors** — 32 closed, 8 declared |
| post-integration harness (wrapper only) | **8,334 checked, 0 failed** | the `ErrVar` reverse copy removed from the wrapper, rebuilt between edit and run: **4,167 of 8,334** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | whole-unit no-op moves **0** — the unit is DEAD; same-build control moves **1,857,893** |

**THE REFERENCE IS NOT A FUNCTION OF ITS ARGUMENTS ON SIX OF ITS TEN OUTPUTS.**
`real(C_DOUBLE) :: setpoints(8)` is never assigned in the configuration this
campaign compiles — `ZMQ_CLIENT` is undefined, because `pkg_check_modules`
does not find libzmq — and the procedure copies it into eight `LocalVar%ZMQ_*`
fields. Measured three calls in one process: `NaN` and denormals on a fresh
frame, `1.0` after a routine that fills that stack region with `1.0`, `-7.25`
after one that fills it with `-7.25`. The harness saw the same thing from the
other side before anything was stated: **4,175 of 4,179 cases agreed on every
output and 4 disagreed on these fields alone**, the reference answering
`68bb11a718b90000` — a leftover pointer. `harness/ranges.toml` grew a second
kind of entry for it, `no_oracle`, which is not a range: a range narrows an
INPUT domain, this says an OUTPUT has no answer. It is reported in the run's own
artifact (`no_oracle_outputs`) and a name matching no field is a hard error.

**4,175 OF 4,179 CASES ENTERED NOTHING, AND THE FIX WAS NOT MORE CASES.** The
whole body sits inside `MOD(LocalVar%n_DT, CntrPar%n_DT_ZMQ) == 0 .OR.
LocalVar%iStatus == -1` — a rate gate, true only on the multiples of a second
varied parameter, which no ladder reaches and which the two-sided-predicate rule
cannot cross either. New rule **R9** re-runs EVERY existing case with the gate
satisfied; 120 fresh cases moved the score by 0.000 first, and so did a strided
sample, because **a sample of a corpus is not a sample of its conjunctions**.
0.2593 → 1.000, with 32 of the 40 survivors closed rather than declared.

**THREE INSTRUMENT DEFECTS FIXED, NONE WORKED AROUND (X2).** `vit
test-validate` emitted a bridge that would not compile — it declares POINTERs to
NESTED types the reference's own `USE ... ONLY` list omits, and every earlier
unit escaped it only because its module re-exported `ROSCO_Types`. `compare_op`
was mutating the angle brackets of `static_cast<...>`: 10 of 30 mutants failed
to build, 33%, above the 25% at which `vit_mutate` REFUSES to score — a guard
meant to catch a build with two definitions of the function, spent on
punctuation. And R9 above. Details in DECISIONS.md and the RUNBOOK's target
layer.

**TWO MORE UPSTREAM ROSCO DEFECTS.** A full-width `ZMQ_CommAddress` aborts the
controller (`CHARACTER(256)` plus `C_NULL_CHAR` into a 256-character record →
`End of record`, exit 2; the harness hit it too). And `n_DT_ZMQ` is `NINT(
ZMQ_UpdatePeriod/DT)` where `ZMQ_UpdatePeriod` is parsed only when `ZMQ_Mode /=
0` — so the divisor is 0 on every input this campaign carries, and the
`ZMQ_Mode > 0` guard on the call is the only thing between the shipped
controller and a division by zero.

---

**As of 2026-08-12: unit #20 `StateMachine` is `integrated` and CLOSED**, first
dispatch — its section is directly below. Unit #19 `SecLPFilter_Vel` is `integrated` and
CLOSED, first dispatch. **Five layers, all five alive** — the fourth unit in
this campaign of which that is true, after #11, #13 and #18.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 27 | 14,818 field rows, ALL `IDENTICAL` | zero stub **0/62**, 477 rows moved |
| differential harness vs clean Fortran | **2,884** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **2,884 of 2,884** |
| mutation score | **79 of 79 behavioural killed, 1.000**, 0 declared equivalent, 2 no-compile | `assoc_reorder` ran and left nothing alive — on the corpus unit #18 widened, not on a new one |
| post-integration harness (wrapper only) | **2,884 checked, 0 failed** | `CornerFreq`/`Damp` swapped at the `seclpfilter_vel_c` call, rebuilt between edit and run: **1,100 of 2,884** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | ×1.000001 moves **14,140 of 5,252,000**, matching no other committed red test |

**A STUB WHOSE ENTIRE RETURN VALUE IS THE CONSTANT `0.0` PASSED THE KERNEL
VERDICT 62/62 (C12).** Not a build defect this time — arithmetic. This is the
VELOCITY form of #18's filter, so it is a differentiator, and at the only call
site in the tree its input is a step that settles: by the captured window the
reference output has decayed to **-2.97e-52**. `kgen_tolerance` is `1.D-14` and
**ABSOLUTE**, so the 63 field rows the stub does move come back
`NOT IDENTICAL(within tolerance)` and are **counted as passes**. The field log
is not blind — the shipped translation's 14,818 rows are all `IDENTICAL` and the
stub's are not — but the verdict line is, which is unit #3's rule in its
starkest sighting. The wrong artifact is committed at
`evidence/SecLPFilter_Vel/kernel.zero-output-stub-PASSES-verdict-63-rows-move.run.txt`.
NOT FIXED HERE: changing what `vit verify` calls a pass is X3, eighteen
committed artifacts wide. Recorded in DECISIONS.md as a candidate, with the
addition that would close it without moving any of them.

**41 OF THE 62 CAPTURED CASES ARE VACUOUS IN THE OUTPUT**, and the same
sentence explains the scenario choice. The only call site is inside
`CableControl`'s `CC_Mode == 1` branch, whose `CC_DesiredL` assignments are
guarded by `IF (LocalVar%Time > 500)`; before that the filter is handed 0 with
all histories 0 and returns 0 for *any* coefficients. Coverage offered
scenarios 3, 7 and 27; scenario 3 was rejected on `Examples/vit_sim.py`'s own
comment — *"with CC_Group_N=1 and tlen=400 < 500, the filter processes zero
input the entire run"* — which is P9 written into the source by a previous
author. Scenario 27 runs to 600 s and 21 of its 62 captured cases are past the
step.

**THE MUTATION SCORE WAS INHERITED, NOT RE-EARNED, AND THAT IS THE RESULT.**
Unit #18 spent its cycle proving two `assoc_reorder` survivors killable and
adding the hot rungs UNPINNED plus `±sqrt(DBL_MAX)` to `harness/generate.py`,
because `2.0*DT**2.0*CornerFreq**2.0` is a bare product with no term to remove.
This unit's `lpfV_a1` is that identical expression. It scored **1.000 on the
first run** with no corpus work at all — the first time in this campaign a
corpus addition has paid for a later unit rather than for the one that made it.

---

**As of 2026-08-12: unit #18 `SecLPFilter` is `integrated` and CLOSED**, first
dispatch. **Five layers, all five alive** — the third unit in this campaign of
which that is true, after #11 and #13.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 14,818 field rows, ALL `IDENTICAL` | zero stub **0/62**, 404 rows moved — but see below: the FIRST run of that stub said PASSED |
| differential harness vs clean Fortran | **2,884** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **2,884 of 2,884** |
| mutation score | **80 of 80 behavioural killed, 1.000**, 0 declared equivalent, 2 no-compile | the inherited corpus scores **0.975, two `assoc_reorder` survivors** — see below |
| post-integration harness (wrapper only) | **2,884 checked, 0 failed** | `CornerFreq`/`Damp` swapped at the `seclpfilter_c` call, rebuilt between edit and run: **1,040 of 2,884**; revert, rebuild, green returns |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | ×1.000001 moves **1,349,326 of 5,252,000**, matching no other committed red test — this unit's own signature |

**A ZERO STUB THAT READS NO ARGUMENT PASSED THE KERNEL 62/62, AND THE CAUSE WAS
A MISSING MAKEFILE PREREQUISITE (C12).** VIT writes the TRANSLATION to
`<stem>.hpp` and a three-line `extern "C"` wrapper to `<stem>.cpp`, and the
generated rule was `seclpfilter.o: seclpfilter.cpp` — so `make` reused an object
built from the real translation and the stub was never compiled. `vit verify`
never sees it (`_build_and_run_kernel` runs `make clean` first); the two RUNBOOK
recipes that hand-run `make` in the kernel directory do, and both are how a stub
or an original-Fortran replay gets re-run. Fixed in VIT `5ba5e7e` by addition,
measured in both directions. **No committed artifact is affected** — every
kernel green in this campaign came through `vit verify`.

**THE MUTATION SCORE REACHED 1.000 BY TWO CORPUS ADDITIONS, AND NEITHER MUTANT
WAS EQUIVALENT.** Both survivors were `2.0*(DT*DT)` → `(2.0*DT)*DT` — unit #13's
`assoc_reorder` shape in a sibling expression — and both were proven killable
over the reachable inputs before anything changed. The hot rungs
(`1e-156`/`1e-158`/`1e-160`) lived ONLY in the joint block, which pins every
other defaulted real to `0.0` or `1e300`; that is right for `NotchFilter`, whose
coefficient is a SUM, and wrong here, where `FP%lpf2_b1` is a bare PRODUCT with
no term to remove. Running the hot rungs UNPINNED took 0.975 → **0.988**. The
`a1` mutant survived that too, because `~1e-306 - 8.0` is exactly `-8.0` under
both spellings; of the **1,936** ladder-by-ladder pairs exactly **six** separate
them and all six are `(hot rung, ±sqrt(DBL_MAX))` — the largest x whose square
is still FINITE. Adding it as a third isolating pin took 0.988 → **1.000**.
2,284 → 2,884 cases (`translation-loop` `9ee71de`).

**THE KERNEL CANNOT CONSTRAIN FOUR OF THE TEN ARGUMENTS**, and the sharp form is
a count: the six `lpf2` coefficient arrays are the only fields written inside
`IF ((iStatus == 0) .OR. reset)`, and the zero stub moves each in **1 of 62**
cases — so 61 of 62 cases do not READ `CornerFreq` or `Damp` at all. A stub
writing the literals the simulation carries passes 14,818 of 14,818.

---

**As of 2026-08-12: unit #17 `Read_OL_Input` is `blocked`**, SECOND dispatch, and
it is the campaign's **first `respecify` unit** and the **first REFUTED plan
prediction**.

**THE SECOND DISPATCH CLOSED P11 AND PRODUCED P12; THE UNIT IS NOW BLOCKED ON
THE SCORE ITSELF RATHER THAN ON A MISSING INSTRUMENT.** The three unmet
conditions were `P12:mutation_missing`, `P11:harness_not_rerun` and
`P13:respecify_unscored`. P11 passes (740 checks against the integrated build).
P12 and P13 now fail with a different reason -- `mutation_below_threshold`,
**0.726 against 1.000** -- which is a measurement where there was none.

| layer | result | red-tested |
|---|---|---|
| differential harness (P11), pre-integration | **740 cases, 0 failed**, against the real Fortran | it went red on the way: 80 of 657 cases on a real translation defect, below |
| differential harness, post-integration | **740 cases, 0 failed** | transposing the two allocate-on-return extents in the wrapper fails **106 of 740** |
| mutation score (P12) | **0.726** -- 98 of 135 behavioural, 0 declared equivalent, 29 no-compile excluded, **4 killed by a watchdog** rather than by a compared value | n/a |
| gate, 27 scenarios | 5,252,000 / 0 mismatched, re-run on the rebuilt integrated library | unchanged from the first dispatch |

**BOTH GENERATORS CROSS THE SIGNATURE NOW.** The first dispatch refuted the
prediction for the SHIPPING bridge and left the matrix's actual subject standing:
`test_validate.generate_fortran_bridge` really did emit `Channels(1:n_Channels)`
for a rank-2 ALLOCATABLE and really did not compile. Fixed in VIT (`117bff1`),
not worked around -- the harness bridge now mirrors `interface_gen`, naming its
extents with the same helper, and a fixed-width `CHARACTER(1024)` dummy no longer
gets a `len_` parameter the prototype does not have.

**THE HARNESS FOUND A DEFECT THE KERNEL AND THE GATE COULD NOT.**
`ALLOCATE(Channels(rows, cols))` with `cols < 0` is an extent of ZERO in Fortran,
not a negative extent. The translation returned the raw `NumChannels`, which
would size a `C_F_POINTER` shape by a negative number. **80 of 657 cases.** This
unit allocates on neither of the paths the kernel and the gate reach.

**THE REFERENCE DOES NOT TERMINATE ON THREE INPUTS**, measured with an 8-second
timeout on `Read_OL_Input` itself: an EMPTY file, a COMMENTS-ONLY file, and a
name resolving to a DIRECTORY. A failed `READ(u,'(A)',IOSTAT=IOS) LINE` leaves
`LINE` unchanged under gfortran, so the comment loop never exits. The
translation stops instead, so the two disagree there and no differential case can
be written for it
(`evidence/Read_OL_Input/reference.does-not-terminate-empty-allcomment-directory.txt`).

**37 SURVIVORS, THREE FAMILIES, NONE A SCATTER** -- buffer-size constants and
end-of-record guard positions (undefined behaviour no value comparison can see),
three branches this corpus cannot reach, and one site the reference itself leaves
undefined. Listed with source lines in
`evidence/Read_OL_Input/mutation.37-survivors-with-source-lines.txt`. The remedy
is unit #15's and it has already worked once here: rewriting `scan_real` and
`scan_repeat` to let `strtod`/`strtoul` do the scanning deleted 31 survivors and
moved 0.708 -> 0.726.

--- what the FIRST dispatch recorded, unchanged below ---

**THE SIGNATURE CROSSED.** `plan.json` predicted `bridge_feasible: no` on
`channels: c_assumed_shape_2d`. `vit interface` crosses the rank-2 assumed-shape
ALLOCATABLE INTENT(OUT) by **allocate-on-return** -- `double** Channels,
int* n_Channels_rows, int* n_Channels_cols` plus `vit_free`, with `C_NULL_PTR`
carrying NOT-ALLOCATED -- and `TYPE(ErrorVariables)` crosses as
`C_LOC(ErrVar_view)`. It builds, links and passes the gate. The prediction
measured `test_validate.generate_fortran_bridge` (the differential harness's
Fortran side); `vit integrate` ships `interface_gen`. Both are true of what each
measured, and the RUNBOOK already warned that `bridge_feasible` answers the other
question.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 1 case, scenario 10 | 201 field rows, 200 `IDENTICAL` + **`errmsg` `OUT_TOL`** | the **ORIGINAL FORTRAN** rebuilt into the same kernel fails the same case with the same message against an EMPTY reference -- the mismatch is KGen's, not the translation's |
| differential harness (P11) | **REFUSED** -- `EmitError: C parameter 'OL_InputFileName' is not in the mapped signature` | n/a |
| mutation score (P12) | **NOT PRODUCED** -- there is no harness to mutate against | n/a |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | the whole unit as a no-op **killed exactly scenarios [10, 14, 24]**, 5,252,000 -> 4,992,000; `went_red` counts VALUES and reads false |

**THE KERNEL PASSED A TRANSLATION THAT WROTE NOTHING (C12).** `ErrVar%ErrMsg`
arrives UNALLOCATED, and `vit_populate_errorvariables` published
`C_NULL_PTR / n = 0 / cap = 0` for that case -- P6-correct and **no buffer at
all**, so the reference's own `ErrVar%ErrMsg = <expr>` (a reallocating
assignment) had no C representation. The translation refused the write, printed
to stderr six times, and the kernel scored **200 of 200 IDENTICAL**, because
KGen guards the comparison on `IF (ALLOCATED(var%errmsg))` -- the KERNEL's value.
**An output the translation fails to allocate deletes its own comparison.** The
wrong artifact is committed. Fixed by addition in `vit/view_populator.py`: the
staging buffer is supplied either way and NOT-ALLOCATED moves onto the length as
`n < 0`, the convention this codebase already used for an ALLOCATABLE extent.
Gate re-run after the change: 5,252,000 / 0.

**AND THEN THE FIELD IS STILL NOT COMPARABLE.** With the buffer supplied,
`errmsg` appears and reads `OUT_TOL` against an EMPTY reference: KGen's generated
reader is `READ (UNIT = kgen_unit) var%errmsg` into a field it just deallocated,
with **no length record**, so a deferred-length CHARACTER never round-trips. Not
fixed here -- it changes the state-file format for every type carrying such a
field, including `ReadAvrSWAP`'s and `ExtController`'s committed kernels (X3,
SPEC 8.4). What the kernel actually constrains for this unit is `avifail == -1`.

**WHY IT IS `blocked`, AND THE THIRD REASON IS THE ONE THAT MATTERS.** Two
ordinary generator gaps: `CHARACTER(1024)` is a width compiled into both sides,
so `build_c_params` emits `char*` with no `len_` and `map_signature` refuses it
(unit #10's shape, one declaration over); and `Channels` maps as an INPUT
`real[]` on the +/-1e3 default where the shipping bridge makes it
allocate-on-return. Underneath both: **this unit's principal input is not in its
signature.** Its behaviour is a function of a FILE ON DISK, and no generator here
has a notion of a file-valued input. Fixing the two mappings would produce a
harness that varies a name and never the bytes.

**23 OF THE UNIT'S 126 LINES ARE ALL THAT ANY SCENARIO RUNS.** The clean call
site `ReadSetParameters.f90:778` runs **3 times in all 27 scenarios** -- 10, 14
and 24, the three coverage records as executing no controller code -- and all
three take `.NOT. FileExists`, because `Examples/example_inputs/OL_Mode2_Input.dat`
is not in this tree. Every line from the `OPEN` to the final read loop is dead
campaign-wide.

---

**As of 2026-08-12: unit #16 `ReadAvrSWAP` is `integrated` and CLOSED**, second
dispatch. (The first dispatch integrated it and passed the gate, then ran out of
time with the mutation score unrun, the post-integration harness unrun and the
tree dirty.)

**FIVE LAYERS ALIVE — and two of them cannot constrain half of what the unit
writes.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases | 14,135 field rows, ALL `IDENTICAL` | the no-op stub FAILS — **and the stub with 23 of the 43 output fields DELETED passes 62 of 62** |
| differential harness vs clean Fortran | **27,656** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **27,656 of 27,656**; the 23-field-deleted stub fails 26,198 of 26,198 and names the fields |
| mutation score | ****110 of 110 behavioural killed, 1.000**, 1 declared equivalent, 2 no-compile** | the inherited corpus scores **0.874, 14 survivors** — see below |
| post-integration harness (wrapper only) | **27,656 checked, 0 failed** | **the assumed-size array forwarded as `avrSWAP(2)` at the CALL — the `BIND(C)` interface block untouched, rebuilt between the edit and the run — fails **27,656 of 27,656**; revert, rebuild, green returns** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `GenSpeed` × 1.000001 moves **1,487,557 of 5,252,000**, matching no other committed red test — this unit's own signature |

**23 OF 43 OUTPUT FIELDS ARE INVISIBLE TO BOTH BIT-EXACT LAYERS AT ONCE**, and
it is the largest single blind spot this campaign has measured. A stub with the
whole `Ext_Interface` block plus `VS_MechGenPwr`, `FA_Acc_TT`, `SS_Acc_TT`,
`NacIMU_FA_RAcc` and `FA_Acc_Nac` deleted passes the kernel 62/62 and moves 0 of
5,252,000 at the gate. Three mechanisms — no data at all (`avrSWAP(1001..1018)`
is written by nothing in this tree), data written and then overwritten by
`control_interface.py` before the DLL sees it, and no reader the gate reads —
and only the second is repairable by widening the scenarios. Not repaired here:
it moves `baseline_arrays` and the compared count (X3, SPEC §8.4).

**A LADDER AIMED AT A NAME IS NOT A LADDER OVER THE QUANTITY.** The corpus
CONTAINED `iStatus = 0` — `LocalVar_iStatus` is a scalar integer parameter and
unit #10's decade ladder gave it every boundary — and every one of those cases
was dead, because the unit's first statement is
`LocalVar%iStatus = NINT(avrSWAP(1))` and `_fill_array` puts element 1 of a
3000-element ramp at about **-999 in every case ever generated**. Same for
`NumBl` and `Time`. Second and separate: **two ladders that never cross cannot
reach a branch that needs both** — the pitch-fault loop needs `PF_Mode == 1` and
`NumBl >= 2` and `iStatus /= 0` in the SAME case, and every stage of the
generator sets one parameter and leaves the rest at base. Closed by addition
(`predicate_knobs_from` + an R7 cross-product block, appended last, bounded at
4096 combinations with an all-pairs fallback that reports itself): 26,198 →
27,656 cases.

---

**As of 2026-08-12: unit #15 `PathIsRelative` is `integrated` and CLOSED**, first
dispatch.

**FOUR LAYERS ALIVE; THE KERNEL IS A LOOKUP TABLE AND ITS OWN STUB SAYS SO.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 1 case, scenario 1 | 197 field rows, ALL `IDENTICAL` | constant `.TRUE.` → 1 of 197 `OUT_TOL`, naming `perffilename`; **the constant `.FALSE.` stub, reading no argument, PASSES 197 of 197** |
| differential harness vs clean Fortran | 387 checked, 0 failed, 0 inadmissible | the answer INVERTED → **387 of 387 failed**, naming `vit_result`. A constant could not have been the red test: the corpus answers `.TRUE.` 331 times and `.FALSE.` 56 |
| mutation score | **26/26 behavioural killed, 1.000**, 0 declared equivalent, 0 nocompile | the literal `INDEX` form scores 0.938 — see below |
| post-integration harness (wrapper only) | 387 checked, 0 failed | `LEN(GivenFil)` → `LEN(GivenFil) - 1` at the bridge CALL → **4 of 387**, which is exactly the number of cases whose answer depends on the last character; revert, rebuild, green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **neither direction moved a value** — and the `.TRUE.` direction **killed 24 of the 27 scenarios** |

**A PERTURBATION CAN BE MAXIMALLY VISIBLE AND STILL READ `went_red: false`.**
Forcing the unit to answer `.TRUE.` prefixes `PriPath` onto an already-absolute
`CntrPar%PerfFileName`, and the scenarios do not finish wrong — they **abort**:
`Fortran runtime error: Cannot open file '.../Examples//workspace/.../Cp_Ct_Cq.NREL5MW.txt'`
at `ReadSetParameters.f90:879`. 24 of 27 die; `compared` falls 5,252,000 →
260,000; `mismatched` stays 0, so `went_red` — which counts VALUES — reads false
beside the message *"either the line is never executed by these scenarios, or the
gate cannot observe it"*, which is false of this run. The artifact **as first
taken** is kept at `evidence/PathIsRelative/gate.redtest.as-taken-scenarios-died.json`
(C12). `scripts/gate.py` now also records `perturbation_broke_scenarios` and
prints the accurate message; `went_red` and the exit code are deliberately
unchanged (X3). **No earlier `gate/*.json` has a non-empty `scenarios_failed`**,
so nothing already committed is re-read by this.

**THE TWO CALL SITES ARE ONE LINE APART AND FAIL IN OPPOSITE DIRECTIONS.**
`PerfFileName` is absolute in all 14 inputs, so the answer is `.FALSE.` and the
branch is skipped — a wrong `.FALSE.` changes nothing and a wrong `.TRUE.` kills
the run. `OL_Filename` is the literal `"unused"` wherever `OL_Mode = 0`, so the
answer IS `.TRUE.` and the branch DOES run, 25 times — and the name it builds is
read only when `OL_Mode > 0`, where `vit_sim.py` supplies an absolute path and
the answer is `.FALSE.` again. Forcing `.FALSE.` moves **0 of 5,252,000** with
all 27 scenarios alive. This is the guard that makes unit #6's `GetPath`
invisible, measured from the other side.

**REACHING 1.000 MEANT DELETING SITES, NOT ADDING CASES — the first such unit
since #4.** The reference's three `INDEX` calls written literally, as a function
returning a POSITION, score **0.938**, and both survivors are its loop bound
`len_s - len_sub + 1`: the loosening mutants read **past the end of the buffer**,
so they are undefined behaviour rather than wrong answers, and the position they
return is a quantity nothing downstream reads — all three calls are immediately
compared against `0`. Rewritten as two predicates whose loop bounds carry no
arithmetic, **the same 387 cases kill 26 of 26**. Both harness runs report 387,
which is what says the corpus was never the problem.

---

**As of 2026-08-11: unit #14 `NotchFilterSlopes` is `integrated` and CLOSED**,
first dispatch.

**FIVE LAYERS ALIVE — the third unit here of which that is true, after #11 and
#13.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 27 | 14,508 field rows, ALL `IDENTICAL` | zero stub → 0/62, moving 661 rows that name all 11 of this unit's outputs; **the constant-arguments stub PASSES 14,508 of 14,508** |
| differential harness vs clean Fortran | 4508 checked, 0 failed, 0 inadmissible | the unit as a no-op → **4508 of 4508 failed** |
| mutation score | **84/84 behavioural killed, 1.000**, 0 declared equivalent, 3 nocompile excluded | — |
| post-integration harness (wrapper only) | 4508 checked, 0 failed | `CornerFreq`/`Damp` swapped at the bridge CALL → 2192 of 4508; revert, rebuild, green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 128,918 of 5,252,000** across 15 channels, 0 after revert |

**THE CORPUS COULD NOT CONTAIN A NEGATIVE ZERO, AND ITS OWN DEDUP IS WHY.** One
mutant survived a 4468-case green: `if (CornerFreq < 0)` → `if (CornerFreq <= 0)`
on the saturation guard. The two predicates disagree on exactly one input,
`CornerFreq == 0`, and at `+0.0` they are bit-identical — so magnitude coverage
can never separate them. `_real_magnitude_ladder` and `_ladder` both end in
`list(dict.fromkeys(out))` and `0.0 == -0.0`, so a `-0.0` written into either
list is absorbed by the `0.0` already there. First rung this generator has
gained that could not be added where it belonged. Closed by a signed-zero block
appended last (loop `5d83048`): 4468 → 4508 cases, 0.988 → **1.000**, and the
mutant dies on exactly **2 of 4508**.

**AND THE RETURN VALUE AGREES IN EVERY DIFFERING DRAW.** At `-0.0` the reference
carries the sign into `2.0*DT*CornerFreq_` and the mutant does not, so
`FP%nfs_b2` differs — 36 of 36 draws — but `(+0.0) + (-0.0)` is `+0.0`, so the
value the function RETURNS is bit-identical in all 36. **A harness comparing only
the unit's result could not have killed this mutant at any input.**
`R4_compare_all_outputs` is the whole of the discrimination here. The same
argument covers the translation: writing the saturation as the Fortran's branch
rather than `std::fmax(CornerFreq, 0.0)` is what made the two distinguishable —
`fmax(-0.0, 0.0)` returns `+0.0`, which is the mutant's answer.

**THREE OF THE SIX SCENARIOS THAT RUN IT RUN IT ON ZEROS.** The gate red test
moves channels in scenarios 8, 26 and 27 and in no others; `Examples/vit_sim.py`
injects a synthetic per-blade `rootMOOP` in exactly those three. Scenarios 6, 16
and 18 execute the call site **108,000 times between them on a zero input**. The
two columns agree exactly — unit #2's finding at the level of the gate rather
than of the kernel window, and the check is one grep of the scenario driver.

**WHAT THE KERNEL CANNOT SEE: five of twelve C parameters, and a whole branch.**
The unit has ONE call site in the controller and it passes `Damp` as the literal
`0.7_DbKi`, `Moving` as `.TRUE.` and no `InitialValue`; `CornerFreq` is a rotor
speed, so the saturation assignment never executes. A translation with all five
as literals and the branch deleted scores 14,508 of 14,508 `IDENTICAL`. What the
kernel CAN see, and #13's could not: because `Moving = .TRUE.`, the coefficient
block runs on every call, so all 62 cases read `DT` and `CornerFreq` and
`rotspeedf` has 23 distinct non-zero references.

---

**As of 2026-08-11: unit #12 `NonDecreasing` is `integrated` and CLOSED**, first
dispatch.

**FOUR LAYERS ALIVE, ONE PROVED VACUOUS BY ITS OWN STUB — and the harness layer
did not exist until this unit built it.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 1 case, scenario 1 | 200 field rows, ALL `IDENTICAL` | constant `.FALSE.` → 2 of 201 `OUT_TOL`, naming `avifail` and `errmsg`; **the constant `.TRUE.` stub, reading no argument, PASSES 200 of 200** |
| differential harness vs clean Fortran | 36 checked, 0 failed | the result INVERTED → **36 of 36 failed**, naming `vit_result`. A CONSTANT could not have been the red test — the corpus now holds both answers |
| mutation score | **16/16 behavioural killed, 1.000**, 0 declared equivalent, 0 nocompile | both `<=`→`<` mutants die on 4 cases apiece, and all 4 are order-ladder cases |
| post-integration harness (wrapper only) | 36 checked, 0 failed | `SIZE(Array)` forwarded as `SIZE(Array) - 1` → 4 of 36, which is exactly the number of cases that perturbation can reach; revert, rebuild, green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 1,857,893 of 5,252,000 moved**, 0 after revert |

**THE CORPUS HAD ONE ORDERING IN IT, AND TWO INSTRUMENTS SHARED ONE BLINDNESS.**
`_fill_array` returns a strictly ascending ramp in every case it has ever
produced, so all 25 generated cases answered `.TRUE.` — the same answer the
kernel gives, for the same reason: `NonDecreasing = .FALSE.` has **zero hits in
all 27 scenarios**, so a stub reading no argument and returning `.TRUE.` passes
the kernel 200 of 200. The differential harness exists precisely to not share
the simulation's blind spots, and here it did. Fixed by addition — an ORDER
LADDER fired only for an array the REFERENCE itself subscripts twice in one
statement. 25 cases → 36; the score goes 0.875 → 1.000, and
`evidence/NonDecreasing/order_ladder_kills_the_le_mutant.txt` names the 4 cases
that do it.

**THE GATE'S RED IS A REFUSAL TO START.** 1,857,893 is byte-identical to
`gate/GetWords.redtest.json`, and that is the finding rather than a coincidence:
all three live call sites are `.NOT. NonDecreasing(...)` → `ErrVar%aviFAIL = -1`,
so a `.FALSE.` answer makes the controller reject its own input file — the same
end state as breaking GetWords' word parser. The red test is therefore also its
own same-build control. What the gate constrains is one boolean; nothing about
the array, and nothing about the answer no scenario produces.

**TWO GENERATOR DEFECTS, BOTH RECORDED BEFORE THEY WERE FIXED, BOTH FIXED BY
ADDITION.** `harness/emit.py` emitted `std::vector<double> Array_a(n_Array_a);`
one line before `n_Array_a` was declared — the CHARACTER path's `predeclared`
mechanism, which the numeric path never got; and
`scripts/_integration_shim.py` emitted `int32_t` with no `<cstdint>`, the third
conditional include that generator has needed for the same reason.

**FIRST LOGICAL FUNCTION RESULT IN THIS CAMPAIGN.** `vit interface` assigns an
`INTEGER(C_INT)` to a `LOGICAL`, which gfortran accepts as an extension with a
warning. That conversion NORMALISES rather than bit-copying — measured, not read
(`evidence/NonDecreasing/logical_result_conversion_probe.f90`): 0 → `.FALSE.`;
1, 2, -1 and 256 all → `.TRUE.` with `TRANSFER(L,0) == 1`. Returning 1/0 is
exact.

---

**As of 2026-08-11: unit #11 `LPFilter` is `integrated` and CLOSED**, on the
SECOND dispatch. The first ran all five layers and then ran out of time with the
tree dirty, the state uncommitted, and one evidence reference naming a file that
did not exist — `kernel.hardcoded-cornerfreq-stub.verify_fields.csv`, described
in prose in both `plan.json` and the evidence README while only the stub `.cpp`
was on disk. The second dispatch RAN that verify (it passes, 14,508 of 14,508)
rather than deleting the claim, and committed. `done_check.py`'s
`P5:unresolved_evidence` is what named it.

**FIVE LAYERS, ALL FIVE ALIVE — the first unit in this campaign of which that is
true.** Units #5 to #10 each closed with the gate proved blind to them.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 14,508 field rows, ALL `IDENTICAL` | zero stub → 172 rows `OUT_TOL`, naming every output; VIT's own `InputSignal * 1.00001` DISCRIMINATING; hardcoded-`CornerFreq` stub → 14,508 of 14,508 IDENTICAL, i.e. the kernel cannot see that argument at all |
| differential harness vs clean Fortran | 996 checked, 0 failed | the unit as a no-op → **996 of 996 failed**, naming `vit_result`, six `FP.lpf1_*` arrays and `inst` |
| mutation score | **38/38 behavioural killed, 1.000**, 0 declared equivalent, 2 nocompile | no survivor at any point, no corpus repair needed |
| post-integration harness (wrapper only) | 996 checked, 0 failed | marshalled result × 1.000001 → 656 of 996; the standard ARGUMENT-SWAP perturbation is EQUIVALENT here and stayed green, kept with the reason |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 1,592,059 of 5,252,000 moved**, 0 after revert |

**THE GATE SEES IT, AND THE CALL GRAPH SAYS WHY.** 21 call sites, 18 live,
3,527,912 calls across 23 of the 27 scenarios, and the busiest one produces
`LocalVar%GenSpeedF` — the speed error both the torque and the pitch controller
are built on. Nothing scales it by a gain that is zero in the input files, which
is precisely what made `HPFilter`, one function up the same file, invisible at
all four of its sites.

**TWO INSTRUMENT DEFECTS, BOTH RECORDED BEFORE THEY WERE FIXED, ONE STILL
UNFIXED.**

1. **The differential harness read the INDEX ROLE off the TRANSLATION**, so the
   no-op red test — which contains no subscript — left `inst` an ordinary scalar
   integer. The generator drew it from unit #10's decade ladder up to `INT_MAX`
   and handed it to a REFERENCE that subscripts a `DIMENSION(1024)` array:
   SIGSEGV, *"harness produced no JSON"*, no artifact. The one run that proves
   the harness can fail was the one run it could not complete. Fixed by ADDITION
   in the loop repo — `infer_indexes_fortran` asks the REFERENCE, which is the
   oracle (P7), and runs second so it can only fill a gap the C++ left. The red
   run then came back at 996 cases, the green run's own figure.
2. **`vit_mutate.py` mutates the translation IN PLACE**, so a run killed
   mid-flight leaves the mutant in the tree. One did, and left
   `*inst = *inst + 2;` in the file `vit integrate` reads next. `git status`
   said nothing — the path was untracked either way. Caught by re-running the
   harness for an unrelated reason and getting 996 of 996 failed where the same
   command had passed minutes earlier. NOT yet fixed in the loop repo; the
   RUNBOOK now requires diffing the translation against a saved copy after every
   mutation run. See DECISIONS.md.

---

**As of 2026-08-11: unit #10 `Int2LStr` is `integrated` and CLOSED**, first
dispatch.

**FOUR LAYERS ALIVE, ONE DEAD, AND ONE OF THE LIVE ONES PROVED VACUOUS BY ITS
OWN STUB.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 1 case, scenario 24 | 1/1 `IDENTICAL` on `ol_string` | no-op → `OUT_TOL`; a WRONG constant → `OUT_TOL`; `'X'` padding → `OUT_TOL`; **the RIGHT constant, reading no argument, PASSES 1/1** |
| differential harness vs clean Fortran | 144 checked, 0 failed | no-op → 144/144 failed, naming `Int2LStr_result` |
| mutation score | 14/14 behavioural killed, 1.000, **5 declared equivalent**, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 144 checked, 0 failed | copy-back shortened 11 → 10 → 144/144 failed; rebuilt, green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED TWICE — a COMPLETE NO-OP moved 0 of 5,252,000** |

**THE FIRST FUNCTION RESULT IN THIS CAMPAIGN, AND THE TWO GENERATORS DISAGREED
ABOUT IT.** `CHARACTER(11) :: Int2LStr` is not a C return value: VIT's
`result_is_out_param` turns it into a trailing `char* Int2LStr_result` the caller
owns and the callee blank-fills, and `vit interface` emits exactly that with the
width **compiled into both sides** rather than passed. The DIFFERENTIAL HARNESS
refused the same declaration — `build_c_params` emits no `len_` for a width that
is a literal in the function's own declaration, so `map_signature` reported the
argument unobservable, *"nothing states how many bytes either side may read"*.
That argument is the unit's **only output**. A shipping bridge and no P11/P12
route at all: unit #8's sentence, one type over, and found the same way — by
asking both generators before writing any C++.

**THREE DEFECTS, ALL IN THE DIFFERENTIAL PATH, ALL FIXED BY ADDITION.** The
third is the dangerous one:

1. `map_signature` refused the CHARACTER function result (above). It now maps it
   to a `char[]` with a SYNTHETIC, PINNED width — the shape `_constant_extent`
   already established for an explicit-shape array.
2. `build_call` ASSERTED that a `char[]`'s width is the next C parameter. True of
   a CHARACTER dummy, false of a result. A new `string_fixed` category carries a
   buffer whose width is in the stream and never in the call.
3. **`vit_harness.py` set `result_ctype` for ANY function.** A result that
   already crosses as an out-parameter would have been emitted a SECOND time as a
   return value — `char ret_a = int2lstr(...)` against a wrapper returning void,
   plus a `&ret_b` argument the bridge has no dummy for. It asked `is_function`
   where the question is *does the result come back through the return value*.
   The two generators agree here; this reader of them did not.

**THE GATE IS BLIND, AND THE CONTROL IS WHAT MAKES THAT A FINDING.** Two
perturbations moved 0 of 5,252,000 — the trailing blank fill rewritten, and the
whole unit made a no-op by an early return. GetWords' committed perturbation,
re-run on this same integrated build, moved **1,857,893 of 5,252,000, bit-for-bit
the number `gate/GetWords.redtest.json` carries**. The chain is alive; the unit is
what cannot be seen. `evidence/Int2LStr/gate.control-getwords-perturbed-MOVES.json`.

**WHY, IN ONE GREP: THE ONLY READER IS A `PRINT`.** `Int2LStr` runs TWICE in all
27 scenarios, both in scenario 24. Its result is concatenated into `OL_String`,
whose sole reader in the entire controller is
`ReadSetParameters.f90:772  PRINT *, 'ROSCO: Implementing open loop control for'//TRIM(OL_String)`.
`gate.py` compares the arrays crossing the DLL boundary and never reads stdout.
The other **eighteen call sites are dead**, and structurally so: sixteen build
`ErrVar%ErrMsg` strings and two build a debug-file FORMAT. A unit whose whole job
is rendering numbers into human-readable messages produces output a numerical
gate does not look at — and all 27 scenarios read valid input files, so no error
message is ever built.

**THE KERNEL IS A LOOKUP TABLE, AND IT TOOK FOUR STUBS TO SAY SO.** Unit #6
prescribed two and unit #7 a third; this unit needed a fourth, because the kernel
compares something none of them anticipated. KGen instruments the ASSIGNMENT, so
the compared field is the caller's `ol_string` and **the unit's own result is
named nowhere in the generated `!local verify variables`**. A no-op fails, a
wrong constant fails, `'X'` padding fails — and the RIGHT constant, reading no
argument at all, passes 1/1. The padding stub is kept as the cautionary one: it
looks like proof the kernel sees all 11 bytes, and it only passes through `TRIM`
because `'X'` is not a blank.

**THE MUTATION SCORE REACHED 1.000 IN THREE MOVES, AND ONLY THE LAST IS A
DECLARATION.**

| | score | what changed |
|---|---|---|
| as written | 0.654 | 9 survivors |
| fix the GENERATOR | 0.731 | `Num` was drawn from the default ±1e3 — see below |
| fix the TRANSLATION | 0.737 | three restatements of one buffer bound deleted; 26 mutants became 19 |
| declare 5, with a proof | **1.000** | 4,038,021 inputs, poisoned guard byte, `differ-IN-BOUNDS 0` |

**A SCALAR INTEGER REACHED NO VALUE THIS GENERATOR CHOSE — IT WAS ALWAYS A
UNIFORM DRAW OVER A RANGE NOBODY DECLARED.** `_ladder` and `_literal_values` are
both driven off `reals`; the only thing that ever set a scalar int was
`rng.randint` over the DEFAULT ±1e3. A Fortran default INTEGER is the whole
32-bit domain, and ±1e3 is 0.00005% of it, all of it in the middle. Every branch
this unit has is a branch about the WIDTH of the number, so at `|Num| < 1000` the
width was 1–4 in every case ever generated and the blank run never shorter than
six. An integer DECADE ladder now exists — decade boundaries and the type's
extremes, both signs, `-2147483648` included for itself because it is the one
value with no positive counterpart and is exactly 11 characters wide. 103 cases →
144, and two mutants died to it. Appended last and fired only for a DEFAULTED
scalar int, so it can only ADD cases: the draws every earlier unit was scored on
are byte-for-byte unchanged.

**THE REMAINING FIVE ARE DECLARED, AND PROVED RATHER THAN ARGUED.** All five are
buffer-bound mutants on an 11-byte result — two on a ternary that cannot change
its answer at `Num == 0`, three writing or reading index 11. Unit #8's rule is
that reasoning is not enough here, because whether such a mutant dies depends on
the heap. `evidence/Int2LStr/mutant_equivalence_probe.cpp` sweeps every value with
`|Num| ≤ 2,000,000`, every value within 1000 of every decade boundary to 10⁹ both
signs, and every value within 1000 of `INT_MAX` and `INT_MIN`, against a guard
byte poisoned to `'\x7f'`. That is exhaustive over the only structure the function
has: its behaviour is a function of the decimal STRING, so it can change only
where the digit count does. **differ-IN-BOUNDS 0 for all five.**

**A SEVENTH P9 SHAPE — OR RATHER, THE FIFTH ONE ARRIVING IN SECONDS INSTEAD OF AN
HOUR.** This is unit #7's shape (live consumer, outside what the gate measures),
but reaching it cost one `grep` for the readers of `OL_String` rather than an
afternoon. The accumulated recipe worked as written.

| unit | shape |
|---|---|
| #1 `AddToList` | the line is never executed |
| #3 `ColemanTransformInverse` | an argument is constant in every scenario |
| #4 `Conv2UC` | 1.3M executions, result cancelled by a symmetric consumer |
| #6 `GetPath` | result produced, consumer's guard false wherever it would matter |
| #7 `GetRoot` | result consumed by a live line, into a side effect outside the instrument |
| #9 `HPFilter` | every call site cancelled, by three different mechanisms, two of them a zero gain in the inputs |
| #10 `Int2LStr` | #7's shape: the only live reader is a `PRINT`, and 18 of 20 call sites build error messages no valid input produces |

---

**As of 2026-08-11: unit #9 `HPFilter` is `integrated` and CLOSED**, SECOND
dispatch. The first dispatch was killed by the 7200 s timeout mid-cycle with its
work uncommitted; it had finished the kernel, the pre-integration harness and
their red tests, and none of it was committed. This dispatch re-ran nothing that
was already measured and carried it the rest of the way.

**FOUR LAYERS ALIVE, ONE DEAD — AND THE DEAD ONE WAS PROVED DEAD RATHER THAN
ASSUMED.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 14,508 field rows, all `IDENTICAL` | zero stub → 183 rows `OUT_TOL`; VIT's own red test discriminating (`InputSignal` × 1.00001). **A HARDCODED-`CornerFreq` STUB PASSES 14,508/14,508** |
| differential harness vs clean Fortran | 832 checked, 0 failed | no-op → 829/829 failed, naming `vit_result`, both `FP.hpf_*` arrays and `inst` |
| mutation score | 26/26 behavioural killed, 1.000, **0 declared equivalent**, 1 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 829 checked, 0 failed | return value × 1.000001 → 527/829; `DT`/`CornerFreq` swapped → 85/829; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — a COMPLETE NO-OP moved 0 of 5,252,000** |

**THE GATE IS BLIND, AND THE CONTROL IS WHAT MAKES THAT A FINDING.** A zero from
a red test looks identical to a zero from a broken instrument. So GetWords'
committed perturbation was re-run on this same integrated build and moved
**1,857,893 of 5,252,000 — bit-for-bit the number `gate/GetWords.redtest.json`
carries**. The chain from build to install to 27 simulations to bit comparison is
alive; the unit is what cannot be seen.
`evidence/HPFilter/gate.control-getwords-perturbed-MOVES.json`.

**THREE CALL SITES, THREE DIFFERENT CANCELLATIONS, AND TWO OF THEM ARE A GAIN OF
ZERO IN THE INPUT FILES.** HPFilter runs 892,000 times across 23 of 27 scenarios,
so unit #1's "never executed" is not the answer, and every one of its four
readers RUNS:

| call site | calls | what cancels it |
|---|---|---|
| `Filters.f90:375`, `:376` | 408,000 each | read only by `FloatingFeedback`, which returns `(0 - FA_vel) * Kp_Float`; `Kp_Float` interpolates **`Fl_Kp` = 0.0000 in all 14 `Examples/*.IN`**, never patched |
| `Filters.f90:395` | 64,000 | read only by `PIController(FA_AccHPF, 0.0_DbKi, CntrPar%FA_KI, …)` — proportional gain the literal 0.0, **`FA_KI` = 0.00000 in all 14 inputs**, never patched |
| `Filters.f90:409` | 12,000, scenario 4 only | reaches only `flp_angle_1/2/3`, which are **identically 0.0 across all 4,000 timesteps of scenario 4's baseline**; the 1-DOF sim produces no rootMOOP, and scenario 26 — the one scenario where flp_angle moves — is `Flp_Mode = 3` and never reaches this site |

**A SIXTH SHAPE OF P9, and it is a compound one.** Unit #4's cancelled-downstream
shape assumed one mechanism. Here every site is cancelled and no two share a
mechanism or a scenario, so following the output to a live reader answers
nothing — all four readers run. The question is what **scales** the value between
the reader and a compared channel, and for two of three sites that scale is a
gain that is zero in every input file the campaign owns.

| unit | shape |
|---|---|
| #1 `AddToList` | the line is never executed |
| #3 `ColemanTransformInverse` | an argument is constant in every scenario |
| #4 `Conv2UC` | 1.3M executions, result cancelled by a symmetric consumer |
| #6 `GetPath` | result produced, consumer's guard false wherever it would matter |
| #7 `GetRoot` | result consumed by a live line, into a side effect outside the instrument |
| #9 `HPFilter` | every call site cancelled, by three different mechanisms, two of them a zero gain in the inputs |

**THE FIRST EXTRACTION WINDOW WAS VACUOUS AND WAS THROWN AWAY FOR IT.** On
scenario 27 the real translation and a zero stub BOTH score 14,508/14,508
`IDENTICAL`. Re-extracted on scenario 1, where the zero stub moves 183 rows. Both
artifacts are committed: the discarded measurement is what makes the kept one a
statement rather than a hope.

**A CONSTANT-ARGUMENT BLINDNESS THE KERNEL CANNOT SEE, unit #3's shape again.**
`CntrPar%F_FlHighPassFreq` is 0.01042 in all 14 inputs, so a translation that
ignores `CornerFreq` and writes the literal passes the kernel outright
(`kernel.hardcoded-cornerfreq-stub-PASSES.verify_fields.csv`). And
`has_InitialValue` is 0 at all four call sites in all 27 scenarios — **the
`PRESENT(InitialValue)` branch is reached by no simulation at all.** The
differential harness is the only instrument that varies either, and it is the
only reason the `negate_cond` mutant on that branch dies.

**FIRST UNIT IN THIS CAMPAIGN THAT NEEDED NO INSTRUMENT REPAIR TO REACH 1.000.**
26 of 26 behavioural mutants killed, no survivors at any point, no equivalence
declared. Units #4, #5, #7 and #8 each spent between one and five instrument
fixes getting there.

**ONE VIT DEFECT, FIXED IN VIT (`300da9c`), NOT WORKED AROUND.** The
`test-validate` bridge dropped the OPTIONAL argument's `PRESENT()` flag and a
procedure-scope `USE`: it emitted a 9-argument `hpfilter_f90` while the generated
C++ test called it with 10, and it passed `InitialValue` unconditionally to an
OPTIONAL dummy. Both wrong artifacts are kept under
`evidence/HPFilter/vit_defects/` per C12 — recorded before the fix, not after.

**`C_LOC(FP)` ONTO A FLAT STRUCT IS SOUND HERE ONLY BECAUSE OF WHAT THE TYPE IS.**
`TYPE(FilterParameters)` crosses as a raw pointer rather than a view populator.
That works because it is 46 fixed-size `REAL(DbKi), DIMENSION(1024)` fields with
no ALLOCATABLE and no possible padding — and the field ORDER was verified
name-by-name against `ROSCO_Types.f90` before the translation was believed. A
reordering would compile, link, and silently read the wrong array.

---

**Unit #8 `GetWords` is `integrated` and CLOSED**, first
dispatch.

**EVERY LAYER IS ALIVE, INCLUDING THE GATE — THE FIRST TIME IN THIS CAMPAIGN.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 62/62 `IDENTICAL` on `words` | no-op → 62/62 `OUT_TOL`; a CONSTANT stub → 1/62; the real translation with `Words(1)` corrupted → 62/62 `OUT_TOL`, so every ELEMENT of the array is compared |
| differential harness vs clean Fortran | 1370 checked, 0 failed | no-op stub → 1343/1370 failed, naming `Words`; green restored |
| mutation score | 57/57 behavioural killed, 1.000, **1 declared equivalent**, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 1370 checked, 0 failed | copy-back stops one element short → 1323/1370 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST PASSED — 1,857,893 of 5,252,000 moved** |

**THE GATE SEES THIS UNIT, AND THE REASON IS THE MIRROR OF UNIT #4's.** `FindLine`
upper-cases `Words(WordInd)` and compares it against the parameter name the
caller asked for — one operand is this unit's output, the other is not. `Conv2UC`
was invisible through 1.3M calls precisely because BOTH its operands went through
it. So the campaign now has a positive case beside its five negative ones, and
one test decides all six: follow the output to its consumers and ask whether
anything the gate reads depends on it ASYMMETRICALLY.

Stated beside the green: **both extents are constant in all 27 scenarios**
(`len_Line` = 2048, `len_Words` = 200, `NumWords` ∈ {2, AryLen+1}). The kernel
inherits all three from the simulation. A defect that only appears at another
width is outside both bit-exact layers; only the differential harness varies
them. Unit #3's constant-argument shape, at EXTENT granularity.

**THE UNIT HAD NO DIFFERENTIAL HARNESS AT ALL, AND ITS ONLY OUTPUT WAS THE
REFUSED ARGUMENT.** `CHARACTER(*), INTENT(OUT) :: Words(NumWords)` is two extents
where `char[]` had one — the gap recorded as known since unit #4, which fell due
here. **`vit interface` handles this shape while the harness generator refused
it**: two generators over the same declaration, disagreeing, and only running
both showed it. Built rather than routed around (loop `6d13949`); rank ≥ 2 is
still refused with its reason.

**THREE MORE CORPUS BLIND SPOTS, ALL THREE NAMED BY A SURVIVING MUTANT.** The
score went 0.889 → 0.905 → 0.921 → 0.983 → 1.000 and no step was found by reading
the generator. Second unit running where that is true.

1. `''` inside a `'...'` literal is ONE APOSTROPHE in Fortran; the miner read the
   unit's own separator set as two literals and lost the character.
2. The seventh separator is `CHAR(9)` — a tab cannot be written as a literal at
   all — and `char_corpus` filtered to printable ASCII, so it was unreachable
   twice over. **The miner's blind spot was exactly the class of character that
   most needs mining.**
3. No string shape put a word anywhere but position 1, so every predicate about
   where a word BEGINS had one answer in every case.

None of the three moves an earlier unit's corpus (22, 26, 26 — recomputed,
unchanged).

**A MUTATION SCORE CAN FLAP, AND THE RUN THAT READS 1.000 CAN BE THE ONE THAT
MEASURED LEAST.** Three IDENTICAL runs scored 0.983, 1.000, 0.983. The mutant
responsible differs from the original only past the end of a buffer — proved
exhaustively, 4,908 disagreements and **zero in bounds**. Two fixes: the
declaration is now a statement about the MUTANT rather than about one run, and a
declared mutant that is killed anyway lands in `declared_but_killed` instead of
being absorbed (loop `5b40e1c`).

**THE REVISION STAMP WAS MIS-ATTRIBUTING ITS OWN OUTPUT, IN THREE PLACES.**
`vit-dev` has no git, so both loop scripts fell through to hand-written,
gitignored pin files. **Unit #7's committed harness and mutation artifacts stamp
`99b57ab-pinned` while the tree was at `0e92a72` — the commit that unit's OWN
corpus fix went in as** — and every artifact since unit #5 says `8c34ceb-pinned`
against a VIT at `87a3847`. `.git/HEAD` is plain text and needs no binary, so
both scripts read it first now and report `-nogit`. And **this campaign's own
`scripts/_harness_stamp.py` was a third site that OVERWROTE the correct read with
the stale pin.** Unit #7's artifacts are not restamped: a verdict that was
correct when it was taken is worth more than a deletion.

---

**Unit #7 `GetRoot` is `integrated` and CLOSED**, first
dispatch.

**THE KERNEL IS A MIRROR AND THE GATE IS BLIND, AND THE INSTRUMENT THAT DOES
CONSTRAIN IT WAS ITSELF BLIND UNTIL THIS UNIT FIXED IT.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 62/62 `IDENTICAL` over the full `CHARACTER(8)` | **a NO-OP also scores 62/62 `IDENTICAL`**; a WRONG-constant stub scores `OUT_TOL`, which is what says the comparison is alive |
| differential harness vs clean Fortran | 726 checked, 0 failed | no-op stub → 700/726 failed, naming `RootName`; green restored |
| mutation score | 60/60 behavioural killed, 1.000, **2 declared equivalent**, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 726 checked, 0 failed | `LEN(RootName)` → `LEN(RootName) - 1` in the CALL → 596/726 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

8 of the 60 mutation kills are CRASHES rather than case mismatches, so the
killed-by-comparison count is 52 of 60.

**THE CALL SITE ALIASES ITS TWO ARGUMENTS, AND THAT BREAKS TWO OF THE
CAMPAIGN'S OWN RED TESTS.** `DISCON.F90:67` is `CALL GetRoot(RootName,RootName)`
— one variable as both the INTENT(IN) and the INTENT(OUT) dummy. KGen's captured
input is therefore bit-for-bit its captured reference output (the state file is
`vit_sim1vit_sim1`), so a stub that reads nothing and writes nothing PASSES. And
because every scenario's name contains no `'.'`, `GetRoot` is the IDENTITY on
the exercised domain — so the gate's standard no-op perturbation is not a wrong
implementation either. Both no-op red tests are green for reasons that have
nothing to do with observability. The tests that carry the claim are a
**wrong-constant** kernel stub (62/62 `OUT_TOL`) and a **wrong-output** gate
perturbation (0 of 5,252,000 moved, `replacements: 1`, `revert_verified: true`).
Unit #6's recipe read forwards gives the wrong answer here; RUNBOOK now says so.

**A FIFTH SHAPE OF P9: the result is consumed, by a live line, into a channel
the gate does not read.** `RootName` has six reader sites and all six build a
FILENAME. Five are dead in all 27 scenarios; the sixth runs 24 times and is
`OPEN(unit=UnDb, FILE=TRIM(RootName)//'.RO.dbg')`. `gate.py` compares the `.npz`
arrays `vit_sim.py` collects across the DLL boundary and never opens a `.RO.dbg`.
Perturbing this unit renames a file.

| unit | shape |
|---|---|
| #1 `AddToList` | the line is never executed |
| #3 `ColemanTransformInverse` | an argument is constant in every scenario |
| #4 `Conv2UC` | 1.3M executions, result cancelled by a symmetric consumer |
| #6 `GetPath` | result produced, consumer's guard false wherever it would matter |
| #7 `GetRoot` | result consumed by a live line, into a side effect outside the instrument |

**THE FIRST DIFFERENTIAL HARNESS REPORTED `224 checked, 0 failed` AND HAD NEVER
EXECUTED THE BRANCH THE PROCEDURE EXISTS FOR.** Three independent gaps in the
generator, each found from a SURVIVING MUTANT rather than from the verdict:

1. R6 mined **single-character** literals, so the character SET `'\/'` was
   invisible and the corpus contained **no backslash at all**. Now every
   character of every literal handed to `INDEX`/`SCAN`/`VERIFY` is mined.
2. The corpus is each literal plus its **collating neighbours**, laid down in
   corpus order — and `'/'` is `'.'+1`, so every generated string containing a
   dot had a separator directly after it. The rule that makes the corpus
   relevant is the rule that blinded it. Planted characters and planted PAIRS
   of the reference's own literals now break the adjacency.
3. The length ladder `{1, N, N+5}` had no **2**, the smallest non-degenerate
   string and the first length at which a first-versus-second-character test has
   two answers.

224 cases / mutation 0.648 → 726 / 0.968. Fixed in the loop repo (`0e92a72`).

**TWO KGEN DEFECTS, BOTH FIXED IN KGEN (`4457cd2`).** The generated kernel
compiled, ran, printed `62/62 passed` and compared **nothing** — VIT's own
"kernel compared 0 output variables" is what caught it. `update_state_info` found
an argument's position with `list.index`, and `Fortran2003.Base` compares nodes
by content, so both occurrences of a repeated actual argument resolved to
argument 0. Fixing that exposed a second: the VERIFY name generator embeds the
declaration's selector in a procedure name, so `CHARACTER(LEN=size(avcoutname))`
produced `kv_discon_character_size(avcoutname)_`. `c839e1a` had fixed exactly
that on the GENCORE side and the VERIFICATION side never got it. Red-tested in
both directions; the green control is a `GetPath` re-extraction that comes back
byte-identical with and without the patch.

**REMOVING AN UNOBSERVABLE RESTATEMENT AGAIN — and this time it is the SAME
INTRINSIC unit #4 removed.** Transcribing `LEN_TRIM` literally scored 0.886 with
**six of eight survivors inside the helper**, and three of those six are
out-of-bounds reads, which cannot honestly be declared equivalent. The
translation carries a three-part proof that `LEN(GivenFil)` serves at both sites.
The two mutants that remain are declared equivalent with their proofs committed
at `mutation/GetRoot.equivalences.md`: an out-of-bounds read no value comparison
can see, and `RootName = ''` — **dead code in upstream ROSCO**, unreachable
because the special case at the top of the procedure has already returned on the
only input that could get there.

---

**Unit #6 `GetPath` is `integrated` and CLOSED**, first
dispatch.

**BOTH BIT-EXACT LAYERS ARE VACUOUS FOR THIS UNIT, AND EACH SAYS SO IN AN
ARTIFACT.** The gate is blind and the kernel has one case that a lookup table
passes. What verifies `GetPath` is the differential harness and the mutation
score.

| layer | result | red-tested |
|---|---|---|
| kernel replay, **1 case**, scenario 1 | 1/1 `IDENTICAL` over the full `CHARACTER(1024)` | no-op stub → `OUT_TOL`. **AND A CONSTANT STUB PASSES 1/1** |
| differential harness vs clean Fortran | 236 checked, 0 failed | no-op stub → 234/236 failed, naming `PathName`; green restored |
| mutation score | 25/25 behavioural killed, 1.000, 0 declared equivalent, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 236 checked, 0 failed | `LEN(PathName)` → `LEN(PathName) - 1` in the CALL → 199/236 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

4 of the 25 mutation kills are CRASHES rather than case mismatches, so the
killed-by-comparison count is 21 of 25.

**A FOURTH SHAPE OF P9, AND IT IS A PROPERTY OF THE CONSUMER.** `GetPath` is not
dead: `ReadSetParameters.f90:331` runs 28 times across the 27 scenarios, and so
do both readers of its output. The result is **produced and never consumed**.
`PriPath`'s only two readers are guarded by `PathIsRelative`, and the guard is
false exactly where the answer would matter — `PerfFileName` is one absolute path
in all 14 `Examples/*.IN`; `OL_Filename` is the literal `"unused"` (relative, so
the concatenation happens) in the scenarios where `OL_Mode` is 0 and nothing
opens it, and absolute in the three that do. Six units, four shapes, and
`coverage/line_coverage.json` can express none of them:

| unit | shape |
|---|---|
| #1 `AddToList` | the line is never executed |
| #3 `ColemanTransformInverse` | an argument is constant in every scenario |
| #4 `Conv2UC` | 1.3M executions, result cancelled by a symmetric consumer |
| #6 `GetPath` | result produced, consumer's guard false wherever it would matter |

Two gate red tests are committed and the record says which one carries the
claim: the no-op (decisive) and the `I == 0` branch forced off (deliberately
weak — coverage already showed that branch has zero hits in all 27, so it is
RUNBOOK's "attempt 1" reproduced on purpose).

**THE KERNEL IS STRUCTURALLY ONE CASE.** The call site runs ONCE PER PROCESS, so
no invocation window can widen it, and every scenario builds the argument as
`os.path.join(this_dir, '<one of 14 DISCON*.IN names>')` — one answer in all 27.
Measured, not argued: a stub reading NEITHER input and writing that literal
blank-padded scores **1/1 IDENTICAL**
(`evidence/GetPath/kernel.constant-stub-PASSES.verify_fields.csv`). Unit #2's
all-zero window in a new costume, and unlike #2 there is nothing to widen.

**A KGEN DEFECT WAS FIXED IN KGEN, AND IT IS ON THE CRITICAL PATH FOR MUCH OF
WHAT REMAINS.** The generated kernel would not compile: KGen hoists the enclosing
procedure's dummies into the driver's PROGRAM scope, and
`ReadControlParameterFileSub` declares `CHARACTER(accINFILE_size) ::
accINFILE(accINFILE_size)` — an AUTOMATIC length, legal on a dummy and illegal on
a local. KGen already had the machinery for `CHARACTER(*)` (`c839e1a`) and keyed
on `selector[0] == '*'`. Fixed additively with a narrow new predicate, and
red-tested in both directions: the pre-fix driver plus gfortran's diagnostic is
kept, and a re-extraction of `Conv2UC` — whose enclosing `FindLine` has both a
`CHARACTER(*)` dummy and `CHARACTER(MaxParamLength)` locals, the two shapes the
predicate must treat differently — regenerated **byte-identical to unit #4's
committed kernel apart from its timestamp** and re-verified 62/62 IDENTICAL.
`ReadControlParameterFileSub` encloses the `ParseInput_*`, `ParseAry`, `FindLine`
and `GetWords` call sites, so any unit extracted from there would have hit it.

**Removing an unobservable restatement raised the score from 0.882 to 1.000 —
the fourth time in six units, and the first RE-measurement of unit #1's exact
rule.** Both loops in `char_assign` written 0-based left two `<` → `<=` survivors
that write one byte past a buffer or into a byte the next loop overwrites —
neither a wrong answer, so no value comparison can see either. Written 1-based,
the way the Fortran states the assignment, the same mutant leaves a byte
UNWRITTEN and both die. The mutant count went UP, 17 → 25: the observable form
has more sites, not fewer.

---


**Unit #5 `ExtController` is `integrated` and CLOSED**, on its
third dispatch. Two dispatches closed it `blocked`; both blocking claims are now
refuted by measurement, and the second one cost less than the first.

**A DEAD UNIT WITH A DERIVED-TYPE SIGNATURE CLOSES THE SAME WAY `AddToList` DID.**
The gate is blind to it — 0 of 28 executable lines in all 27 scenarios, and
making the unit a no-op moves 0 of 5,252,000 values — so the evidence is the
differential harness and the mutation score, against an oracle this campaign
had to build:

| layer | result | red-tested |
|---|---|---|
| kernel replay | NOT ATTEMPTED — no live call site, so nothing to capture | n/a; unit #1's rule |
| differential harness vs clean Fortran | 163 checked, 0 failed | no-op stub → 163/163 failed; green restored |
| mutation score | 48/48 behavioural killed, 1.000, 4 declared equivalent, 8 nocompile excluded (13.3%) | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 163 checked, 0 failed | ErrVar reverse copy removed → 163/163 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

**THE DEFECT THIS UNIT FOUND IS BIGGER THAN THE UNIT.** `vit test-validate`'s
decomposed bridge copied every field of a derived-type argument IN and almost
none OUT. A rank-1 ALLOCATABLE field and a SCALAR field of an INOUT argument
were both **input-only**, silently — so `ExtDLL%avrSWAP`, which is the whole of
what `ExtController` produces, and `ErrVar%ErrStat`, which is where ROSCO puts
every error status, were handed to the reference and then discarded. **Every
unit in this campaign taking a derived type was affected**, which is most of
them. Fixed in VIT (`8c34ceb`) as three members — pointer, extent BY REFERENCE
because the callee chooses it, and a capacity — with an over-long result
REFUSED and reported rather than truncated. The wrong artifact is kept at
`evidence/ExtController/vit_defects/extcontroller_bridge.alloc_input_only.f90`.

Four further findings, each measured:

1. **A CHARACTER FIELD did not cross, and 37 of 69 units take one.** `vit
   test-validate` refused a deferred-length `CHARACTER(:)` field outright and
   the loop's `expand_derived` dropped every CHARACTER field with a comment
   saying so. Both closed (VIT `8c34ceb`, loop `7d7c913`). A CHARACTER **array**
   field is still refused, with its reason.
2. **`ExtDLL%avrSWAP(49)` is the constant 2, not the size of the message
   buffer its own comment claims.** `avcMSG` is an ARRAY of `CHARACTER(1)`, and
   `LEN` of a CHARACTER array is the ELEMENT length. Measured before any C++
   was written (`evidence/ExtController/len_probe.txt`) and transcribed, not
   corrected: the original is the oracle. An upstream ROSCO defect.
3. **`avcMSG = TRANSFER(C_NULL_CHAR, avcMSG)` writes ONE byte and leaves the
   rest indeterminate.** gfortran copies min(size) elements from a nonconforming
   right-hand side (`evidence/ExtController/transfer_probe.f90` prints
   `0 90 90 ...`). The translation zero-initialises instead, with the proof
   written in the file — and the oracle fixture deliberately does NOT read those
   bytes, because a stub that folded in indeterminate memory would make the two
   sides disagree for a reason about neither implementation.
4. **`vit integrate --auto-allocate` cannot be used on this codebase.** Its
   copy-call scan matches ANY `CALL x(..., arg%field, ...)`, so it hoisted both
   `LoadDynamicLib` AND the external DLL call into the wrapper; it declines a
   size expression that is a local PARAMETER; and its error handling emits
   `SetErrStat` and `CHARACTER(ErrMsgLen)`, OpenFAST names that exist nowhere
   in ROSCO. Artifact kept; the one ALLOCATE this unit needs is in the committed
   wrapper with the reason beside it. **Open, and it is the campaign's problem
   now, not this unit's.**

**Escalation 3 is CLOSED.** Escalation 2 still stands in part — `loop/done.py`
still has no branch for `integrated_unexercised` — but it no longer blocks
anything: a dead unit whose signature crosses closes as `integrated`, and this
is the second one.

---


**Unit #4 `Conv2UC` is `integrated` and CLOSED.** Every layer green with its own
red test — except the gate, whose red test FAILED and is committed as the
finding.

**THE GATE IS BLIND TO A PROCEDURE CALLED 1,333,146 TIMES.** `Conv2UC` converts
4,558,823 characters across the 27 scenarios; it is among the hottest procedures
in the controller. Two perturbations of the integrated C++ were built and gated
and **each moved 0 of 5,252,000 values** — `ch - 32` → `ch - 31`, and the guard
forced false so no conversion happens at all. The cause is structural: every
value the unit produces is consumed by an equality test against another of its
own outputs (`FindLine` upper-cases both the expected parameter name and the
file's word before comparing them), so a perturbation lands on both operands and
equality survives.

This is the third distinct shape of P9 in four units and the sharpest: #1 was a
line no scenario reached, #3 was an argument constant in every scenario, and
this is an **executed line whose result is cancelled downstream**. A hit count
of 1.3M says nothing about observability, and `coverage/line_coverage.json`
cannot express the difference.

Two further findings:

1. **The differential harness had no string kind, and 20 of 69 units need one.**
   `vit_harness.py` reported the CHARACTER argument comparable-but-held-constant
   and then died on `C parameter 'Str' is not in the mapped signature`. With
   P11 and P12 mandatory, none of those 20 units had a path to close. Built
   rather than worked around: loop `9bee569`. A CHARACTER **array** dummy is
   still refused, with its reason.
2. **Removing a restatement raised the mutation score from 0.696 to 1.000, for
   the second time in the campaign.** Transcribing `LEN_TRIM` literally adds six
   mutable sites computing a quantity nothing downstream can read; five survived
   as out-of-bounds reads no value comparison can see. Unit #1 named a size
   once; this named a loop bound once.

Units #1 `AddToList`, #2 `ColemanTransform` and #3 `ColemanTransformInverse`
remain closed. Escalation 1 is answered; **escalation 2 still stands** — a dead
unit whose signature does NOT cross has no path to harness, mutation or gate,
and `loop/done.py` still has no branch for `integrated_unexercised`.

Next: the lowest-order remaining unit in `plan.json`. Confirm it against
`plan.json` before starting, including its `phase` and `proposed_verification`
fields — those are hypotheses, not facts. Run
`python3.12 scripts/done_check.py <Unit>` before setting any disposition.

## Unit #20 — StateMachine — 2026-08-12

`SUBROUTINE StateMachine(CntrPar, LocalVar)`, `ControllerBlocks.f90`.
**`integrated`.** Five layers, all five alive — the fifth unit of which that is
true, after #11, #13, #18 and #19.

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62, **14,260 field rows all IDENTICAL** | wrong-constant stub 0 of 62, 123 rows |
| differential harness | **3610 checked, 0 failed** | no-op fails 3610 of 3610, naming both outputs |
| mutation score | **39 of 39**, 1.000, 0 declared equivalent | — |
| post-integration | 3610 checked, 0 failed | reverse-copy removed: 3610 of 3610 |
| gate, 27 scenarios | 5,252,000 / 0 mismatched | **RED 36,577**, revert 0 |

The unit is a pure decision tree — nine `LocalVar` reads, six `CntrPar` reads,
two writes, no arithmetic — and its `bridge_feasible: unknown` is **refuted**:
both derived types cross as `C_LOC` onto view structs the campaign already had.
`--reverse-copy` is load-bearing rather than optional, because both outputs are
INTEGER scalars of the INOUT argument; removing that one `CALL` fails the
post-integration harness 3610 of 3610.

**Three measurements worth carrying, all in the RUNBOOK:**

1. **The no-op stub is a MIRROR and passes 60 of 62 kernel cases.** Both
   outputs are fields of the INOUT argument, so they arrive carrying the
   previous timestep's answer, and holding its state is what the unit is for.
   Unit #7's aliasing shape reached through TIME. A right-constant stub passes
   61 of 62 — the window has one answer in all but the initialisation case.
2. **Seven of the thirteen assignment sites are dead in all 27 scenarios**, and
   the unit is nonetheless the most gate-visible one measured so far: a
   whole-unit no-op moves **1,526,538 of 5,252,000** across 22 of 27 scenarios.
   The committed red test moves 36,577, all in scenario 12 — so a red test
   proves visibility and does not measure it. Both artifacts are committed.
3. **The mutation score read 0.769 with nine survivors and none was a
   transcription defect.** A branch-reachability probe put all nine inside two
   sub-trees no case of a 2028-case corpus entered. Two corpus gaps, closed by
   addition in the loop repo (`21ed899`): a predicate written against a named
   PARAMETER produced no knob at all, and a predicate between two VARIED
   quantities has no crossing value in any ladder. 2028 → 2890 → 3610 cases,
   all 16 leaves reached, 39 of 39 killed.


## Counts

40 attempted / **36 integrated** / 0 integrated_unexercised / 0 out_of_scope /
**3 deferred** (unit #29 `CheckInputs`, unit #31 `Debug`, unit #32 `FindLine`) /
**1 blocked** (unit #17 `Read_OL_Input`).

69 units in `plan.json`; 29 remain. 36 + 3 + 1 + 29 = 69.

RECOUNTED at unit #40 from `plan.json` with the one command below, not
incremented. It was **six units stale** (`30 / 35 remain`, last recounted at
unit #34) — the longest this block has ever been wrong, and the sixth time the
unit that fixed it was not the unit that broke it.

RECOUNTED at unit #34's second dispatch from `plan.json` with the one command
below, not incremented. `PIDController` moved OUT of `deferred` and into the
integrated count: its score is 1.000 and `done_check` says so, where at the
first dispatch it was 0.759 and `done_check` said that.

RECOUNTED at unit #33 from `plan.json` with the one command below, not
incremented. It was one unit stale (`28 / 37`), which is the fifth time — and
the fifth time it was corrected by a later unit than the one that broke it.

RECOUNTED at unit #32 from `plan.json`, not incremented — the block was two
units stale again (it still read `27 / 1 / 40` across units #30 and #31), which
is the fourth time, and the fourth time the unit that fixed it was not the unit
that broke it.

**Unit #29 `CheckInputs` moved from the integrated count to `deferred`, and the
count is one lower than it was.** Its translation ships and its green layers are
unchanged -- 23,076 differential cases against clean Fortran, 22,824 against the
integrated build, the gate at 5,252,000 values / 0 mismatched. What changed is
that its mutation score is now a real number instead of an invalid one, and then
instead of an incommensurable one: **0.4509**, all five per-operator parts taken
on the clean tree at loop_rev `2e2295f`, reference side asserted by `nm`, against
a threshold of 1.0. `done_check` fails P12 and the disposition says so rather
than the prose beneath it.

(This block read `8 / 8 / 61 remain` through unit #9, which did not update it.
Recounted from `plan.json` at unit #10 rather than incremented, and recounted
again at unit #11, and again at unit #19 — it had gone stale at `16 / 16 / 53`
across units #17 and #18 — and again at unit #22, stale at `19 / 18 / 50` across
units #20 and #21. Recount, never increment: **twice now the block has gone two
units stale, and both times the unit that fixed it was not the unit that broke
it.** The recount is one command and it is in this file's own instructions:
`python3 -c "import json,collections; print(collections.Counter(u.get('disposition') for u in json.load(open('plan.json'))['units']))"`)

**Twenty of the 33 units with a gate red test are gate-visible.**
RECOUNTED at unit #33 by READING every `gate/*.redtest*.json` and taking the
largest `mismatched` of any that carries `went_red: true`, rather than by
editing this list — which is what the recount instruction above means and which
is how the `StateMachine` discrepancy below was found:

```
saturate 2,255,249 · PIController 2,133,598 · FindLine 1,857,893 ·
GetWords 1,857,893 · NonDecreasing 1,857,893 (shared — see below) ·
LPFilter 1,592,059 · ReadAvrSWAP 1,487,557 · identity 1,462,798 ·
SecLPFilter 1,349,326 · interp1d 1,341,803 · SetpointSmoother 617,546 · NotchFilter 551,278 ·
ColemanTransformInverse 389,644 · sigma 229,165 · wrap_180 206,976 ·
NotchFilterSlopes 128,918 · ColemanTransform 124,353 · wrap_360 84,477 ·
StateMachine 36,577 · SecLPFilter_Vel 14,140
```

`SetpointSmoother` is inserted by VALUE and not appended: **617,546** places it eleventh, between `interp1d` (1,341,803) and `NotchFilter` (551,278). Its figure is the LARGER of its two red tests, the `PC_RefSpd` scaling offset by 1e-6; the other, `R_Total` dropped to 1.0, moves 56,128 — and *that* one moved two scenarios where this campaign's own recorded reasoning predicted one. See `DECISIONS.md`.

`PIController`'s figure is the LARGEST of its three red tests (`error` forced to
zero), not its whole-unit no-op, which moves 1,780,508 — the recount takes the
largest because the question this list answers is whether the gate can see the
unit at all.

The thirteen with a committed red test that stayed green: `AddToList`,
`ChkParseData`, `Conv2UC`, `Debug`, `ExtController`, `GetPath`, `GetRoot`,
`HPFilter`, `Int2LStr`, `PathIsRelative`, `Read_OL_Input`, `UpdateZeroMQ`,
`unwrap`. Two of those are not "the gate could not see a perturbation": `Debug`
writes only a file, which the gate never opens, and `Read_OL_Input` is
`blocked`. `ChkParseData` joined this list after unit #30 and its red test —
`aviFAIL = -7` and an error message on every call — moved 0 of 5,252,000.

**`wrap_360` is the counterexample this list has been waiting for, and it is
the same three statements one screen down.** Its whole-unit no-op moves 84,477
and the stub deleting **both of its arms moves 31,579** — where `wrap_180`'s
identical stub moves 0. One arm of the two is live (15,199 of 19,998 calls take
`x >= 360`, all in scenario 22), so the per-arm numbers are `31,579` for the
high arm and `0` for the low one, and the second zero has its control on the
same build. A per-arm number can be non-zero here; the list below still cannot
carry it.

**`wrap_180` is gate-visible and gate-blind at once, and the list above cannot
say that.** Its red test moves 206,976 values, so it belongs on this list; the
stub deleting **both of its wrapping branches** moves 0 of 5,252,000, so the gate
certifies a translation missing half the unit. A single per-unit number answers
*can the gate see this unit at all* and says nothing about *which parts*. Every
entry above is that same one number, and at least three units now have a second
one worth carrying (`saturate`'s upper clamp, `sigma`'s two clamps, `wrap_180`'s
two branches, `wrap_360`'s low arm). Logged under Open.

**`StateMachine`'s committed artifact does not hold the number this file has
been quoting.** This block read **1,526,538 (whole-unit no-op)**;
`gate/StateMachine.redtest.json` holds **36,577**, from a narrower perturbation
(`VS_State_Region_1_5` → `VS_State_Region_2`, one replacement). Both are real
red tests and the unit is gate-visible either way, so nothing about its
disposition moves. The artifact's number is the one carried above, because the
artifact is what exists. Which run produced 1,526,538, and whether it was
superseded deliberately or overwritten, is **not** resolved here — it is another
unit's evidence and is logged under Open. Found only because unit #26 recounted
from the files instead of incrementing the list.
`identity`'s is the widest in CHANNELS rather than in values — 115 of 351,
across 22 of 27 scenarios — because its result feeds the wind-speed estimator's
covariance, and the estimated wind speed feeds both the pitch and the torque
schedule.
`SecLPFilter_Vel`'s is the smallest yet and the most precisely attributable: the
four moved channels are `cc_actuated_dl` and `cc_actuated_l` in scenarios 7 and
27 and nothing else, 3,999 of 24,000 samples each, which is exactly the tail
after `Time > 500`.

**10 of the 25 integrated units are invisible to the gate** — the 8 below, plus
`UpdateZeroMQ`, which is a ninth reason and the only simple one (never called,
its guard is `ZMQ_Mode > 0` and `ZMQ_Mode` is 0 in all 14 inputs), plus
`unwrap`, which is a **tenth** and is the first with *two* causes in one unit:
one call site is an unentered guard and the other is reached by three scenarios
that a missing input file turns back two statements short of the call.
15 visible + 10 invisible = 25, and that identity is worth checking each time
this block is edited; it did not hold across units #20 and #21.

The 8 are **of the first 16 integrated units**, for seven different
reasons. `ReadAvrSWAP` is NOT one of them — its red test moves 1,487,557 of
5,252,000 — but it carries the campaign's largest *partial* blindness instead:
23 of its 43 output fields move nothing at the gate and nothing in the kernel,
recorded above rather than counted here, because the unit as a whole is visible.
`PathIsRelative` is the newest invisible one and it is a seventh shape: its answer is
`.FALSE.` at the call site where the value would matter and `.TRUE.` at the one
whose value nothing reads, so no wrong answer it can give moves a compared value
— while the wrong answer in the *other* direction does not produce a wrong number
at all, it **aborts 24 of the 27 scenarios**. The remaining six: `AddToList` is
never called, `Conv2UC` is called constantly and
cancelled, `ExtController` is never called *and* has no observable effect on any
channel the gate compares even when it is, `GetPath` is called in every scenario
and its answer is never consumed, `GetRoot`'s answer IS consumed — by a live line
that uses it to name a debug file the gate never opens — `HPFilter` runs 892,000
times with every call site cancelled by a different mechanism, and `Int2LStr`'s
only live reader is a `PRINT`. Each carries a green gate artifact committed beside
the red test that says it constrains nothing.

**And from unit #9 onward each also carries a CONTROL** — GetWords' known-red
perturbation re-run on that unit's own build — because a red test that comes back
green is indistinguishable from a broken instrument without one. The five units
before #9 do not have theirs; that is recorded under Open.

**Unit #11 needs no control**, and that is the point of the rule rather than an
exception to it: `LPFilter`'s gate red test WENT RED (1,592,059 of 5,252,000), so
it demonstrated the chain a control would have been asked to demonstrate. The
gate-visible list is kept once, in the recount above. **Units #16, #18 and #19
need no control either**, for the same reason as #11: their red tests went red.

**Unit #15 needed a control and it is the reason the rule exists.** Both of
`PathIsRelative`'s red tests came back with 0 values moved, and on that build the
GetWords control reproduced 1,857,893 exactly — so the quiet is the unit's, not
the chain's. It also produced the first `gate/*.json` in this campaign with a
non-empty `scenarios_failed`: 24 of 27, which is a perturbation the gate saw in
the only way a killed scenario can be seen. `went_red` cannot express that, and
`perturbation_broke_scenarios` was added beside it rather than in place of it.

**Unit #12's is not a sixth sighting; unit #13's is.** `NonDecreasing`'s
1,857,893 is byte-identical to `GetWords`' — both perturbations end in the
controller rejecting its own input file, which has ONE output signature — so
what the gate constrains there is a single boolean. `NotchFilter`'s 551,278
matches no other committed redtest figure, and it is a third of `LPFilter`'s
because only 6 of the 27 scenarios configure a notch filter at all. Compare the
moved count against every committed artifact before writing "the gate sees this
unit". `NotchFilterSlopes`' 128,918 matches none of them either, and it is
smaller again because only 3 of the 6 scenarios that call it hand it a non-zero
input.

**Every unit now has a `harness/` and a `mutation/` artifact.** `ExtController`
was the exception for two dispatches and the absence was recorded as the
finding; it is now unfinished work that got finished.

## Evidenced

- **E1.2** — `rosco/controller/CMakeLists.txt` passes `-ffp-contract=off` to
  gfortran and to g++, and `scripts/assert_fp_contract.sh` asserts both the
  build line and **zero FMA instructions in the linked library**. Red-tested by
  blanking the flag: 0 build lines, 103 FMA, exit 1.
- **E3.1** — `scripts/gate.py` runs 27 scenarios, compares 5,252,000 values
  across 351 channels bit-for-bit against `baseline_arrays`, prints the count
  and persists `gate/__gate__.json`. Exits non-zero on mismatch and on comparing
  nothing.
- **E3.2** — gate observed red: perturbing `LPFilter`'s leading `1.0` moved
  1,604,573 of 5,252,000 values across 138 of 351 channels; reverting restored
  0. `gate/__gate__.redtest.json`.
- **E3.5** — `baseline_arrays/` regenerated from clean pre-integration source
  after E1.2 changed the flags; 20 of 27 scenario files moved. E3.2 re-run
  against the new set, so the red and green evidence describe the same
  baselines.

Both E3.1 and E3.2 were red-tested **as criteria**, not just satisfied:
corrupting the expected values in `phases.toml` turns them `[FAIL]`, restoring
them turns them `[ok]`.

### The campaign's VIT: `d07a716` → `22086e8` → `37f8bdf` (unit #1) → `f8ab74f` (unit #5)

**`f8ab74f` (unit #5)** makes `generate_fortran_wrapper` say, on stderr, which
derived-type arguments it accepted and did not forward. `dropped_derived_args`
already existed and only `test_validate` asked it, so `vit interface` — the
command that SHOWS you the wrapper — and `vit integrate` — the one that SHIPS it
— both emitted a bridge with fewer arguments than the wrapper, in silence. On
`ExtController` that was `LocalVar` and `ErrVar`, and `LocalVar%iStatus` is the
guard on the entire initialisation branch.

**Additive: no generated byte changes**, so no artifact already measured against
an earlier revision is invalidated — units #1–#4 are unaffected. Red-tested in
both directions (fires on `ExtController` with the strategies unset, silent on
`ColemanTransform` and on `ExtController` once they are set). 937 tests pass,
935 before plus the two added with it.

The three earlier moves happened during unit #1. Neither of the first two was an
upgrade; both are fixes that unit produced.

`22086e8` (first dispatch) made `interface_gen` REFUSE an ALLOCATABLE
INTENT(INOUT) dummy instead of emitting a wrapper that compiles and does
nothing, and gave the conformance matrix an `integrates` column measuring the
generators `vit integrate` actually ships.

`37f8bdf` (second dispatch) replaced the refusal with the feature it was
standing in for: the descriptor bridge, in all three generators
(`interface_gen`, `test_validate`, and the scaffold), plus two things found on
the way — `vit_translated.h` did not include `<ISO_Fortran_binding.h>` for a
declaration that names `CFI_cdesc_t`, and **every file `vit integrate`
generated carried `// After verification: <name> kernel PASSED`**, a verdict
the integrator never checked and which was flatly false here. The loop's
harness learned the same convention in `cf885e3`. VIT suite 935 passed; the
loop's 393, with 7 pre-existing `test_tiny_campaign` failures confirmed present
without these changes.

`tests/conformance/matrix.toml`'s `c_alloc_inout` has now been all three
things: `compiles = "no"` (measured on the wrong generator), `integrates =
"refused"`, and now `yes`/`yes`. Its `why` carries the history.

**`AddToList`'s first-dispatch evidence was deliberately measured under
`d07a716`** and stamps it. `evidence/AddToList/vit_interface.stdout.txt`,
`vit_translate.stdout.txt`, `addtolist.scaffold.cpp` and
`bridge_probe/mod_vit.f90` **cannot be regenerated** — that generator no longer
exists in either later form. They are the record of what it did.

`ColemanTransform`'s committed evidence was produced under `d07a716` and is
unaffected in substance. One caveat, recorded rather than left to be
discovered: `rosco/controller/src/colemantransform.cpp` was NOT regenerated, so
the "regenerates byte-identical" property below no longer holds for its three
header comment lines, which `37f8bdf` changed. Everything else about it is
untouched.

### ColemanTransform, second pass — every number re-measured

Every artifact for this unit now comes from ONE instrument pair, VIT `d07a716`
and loop `ebce989`, and each names its own producer in a `vit_rev`/`loop_rev`
field. The first pass used VIT `d85b33b` and loop `3c88913`.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 27 | 63/63, 14,175 fields IDENTICAL | zero-writing stub → 124 `OUT_TOL`; green restored on revert |
| differential harness vs clean Fortran | 199 checked, 0 failed | mutation, below |
| mutation score | 35/35 killed, 1.000 (33 by case mismatch, 2 by failing to compile) | baseline green first, else it refuses to score |
| post-integration harness (wrapper only) | 199 checked, 0 failed | wrapper args swapped → 199/199 failed; green restored on revert |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `2.0/3.0` → `2.000001/3.0` moved 124,353 values across 15 channels; revert → 0 |

`vit integrate` under the new VIT regenerated `Functions.f90`, `CMakeLists.txt`
and `colemantransform.cpp` **byte-identical** to the committed integration.

### ColemanTransformInverse — every layer, and its red test

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 27, `Controllers.f90:561` | 63/63, `pitcomipc_1p` `IDENTICAL` in every case | zero-writing stub → 61/63 `OUT_TOL`; green restored on revert |
| differential harness vs clean Fortran | 257 checked, 0 failed | the mutation score, below |
| mutation score | 24/24 behavioural killed, 1.000, 0 declared equivalent | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 257 checked, 0 failed | `axTIn`/`axYIn` swapped in the wrapper's CALL → 256/257 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `PitComIPC[0]` × 1.000001 moved 389,644 values; revert → 0 |

2 of the 24 mutation kills are CRASHES (`[2] -> [2 + 1]` and `'2' -> '3'`, both
writing past the caller's 3-element array), so the killed-by-case-mismatch count
is 22 of 24. 2 further `compare_op` mutants did not compile and are EXCLUDED
from the score rather than counted — `vit_mutate.py` changed this after unit #2
(loop `46a7f4f`), so that unit's `35/35` and this one's `24/24` are not the same
measurement.

The signature crosses whole: `PitComIPC(3)` `INTENT(OUT)` becomes
`REAL(C_DOUBLE), INTENT(OUT) :: PitComIPC(*)` — by reference, rank preserved, no
extent parameter emitted or needed — and the five scalars go by `VALUE`.
`vit interface` was read attribute by attribute before any C++ was written, per
the habit unit #1 installed; nothing was dropped.

### Conv2UC — every layer, and the one red test that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 1, `ROSCO_Helpers.f90:1118` | 63/63 `IDENTICAL` | no-op stub → 31/63 `OUT_TOL`; green restored |
| differential harness vs clean Fortran | 118 checked, 0 failed | no-op stub → 27/118 failed |
| mutation score | 14/14 behavioural killed, 1.000, 0 declared equivalent (4 nocompile EXCLUDED) | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 118 checked, 0 failed | `LEN(Str)` → `LEN(Str) - 1` → 24/118 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED, twice — 0 of 5,252,000 moved** |

2 of the 14 mutation kills are CRASHES rather than case mismatches (both index
before the buffer), so the killed-by-comparison count is 12 of 14. The 4
excluded nocompile mutants are all `harness/cppmutate.py` mangling
`static_cast<unsigned char>` — 22% against the 25% at which `vit_mutate.py`
refuses to score.

The signature crosses whole: `CHARACTER(*), INTENT(INOUT) :: Str` becomes
`CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: Str(*)` plus
`INTEGER(C_INT), VALUE :: len_Str`, with copy-in/copy-out in a wrapper that
declares the dummy exactly as the original does. `vit interface` was read
attribute by attribute before any C++ was written.

**For a CHARACTER output a kernel PASS *is* a bit-identity claim** — KGen's
generated comparison has no tolerance branch at all. Unit #3's caveat is a fact
about REAL fields, not about the verdict line; read the generated comparison for
each new element type.

### ExtController — every layer, and the one red test that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT ATTEMPTED** — 0 of 28 lines in all 27 scenarios, so no state to capture | n/a; unit #1's rule is that this is deadness, not a tool defect |
| differential harness vs clean Fortran | 163 checked, 0 failed | no-op stub → 163/163 failed, naming ExtDLL.avrSWAP, ExtDLL.n_avrSWAP, ErrVar.ErrStat, ErrVar.ErrMsg; green restored |
| mutation score | 48/48 behavioural killed, 1.000, 4 declared equivalent, 8 nocompile EXCLUDED | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 163 checked, 0 failed | ErrVar reverse copy removed → 163/163 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

The gate red test carries `replacements: 1`, `perturbed: true`,
`revert_verified: true`, `residual_dirt: []`, and perturbs toward **absence** —
the unit made a no-op — which is the form that shows the gate is blind to the
*unit* rather than to the perturbation chosen. `gcov` says the same thing
independently: **0 of 28 executable lines, in all 27 scenarios.**

**The four survivors are DECLARED, not smoothed over, and each was applied by
hand and re-run before it was declared** (`mutation/ExtController.equivalences.json`).
Two are the same fact: `LEN_TRIM(ErrVar%ErrMsg)` is 0 in every admissible case
because `LoadDynamicLib` blank-fills the message through a `CHARACTER(*),
INTENT(OUT)` dummy, so the concatenation's right operand is constant and the
memmove moves zero bytes. One is a capacity boundary four orders of magnitude
away. One is a buffer length nothing reads back.

**FOUR OTHER SURVIVORS WERE REMOVED RATHER THAN DECLARED, and one was fixed in
the ORACLE.** `accINFILE`'s bound and record 50 are the same expression, as are
`avcOUTNAME`'s bound and record 51; transcribing each twice left a site
computing a quantity nothing reads. Naming each once took the score from 0.758
to 0.904 — the third time in this campaign that removing a restatement raised a
mutation score. The fifth, `aviFAIL = 0`, was a write-only local: the stub now
reports the incoming value back in record 44 before overwriting it, and the
mutant dies. **Where the blindness is in the instrument rather than in the unit,
the instrument is what to change.**

**The oracle fixture had to grow twice for this**, and both times because a
value was reaching the library and never being read: the bytes of `accINFILE`
and `avcOUTNAME` (record 46) and the incoming `aviFAIL` (record 44). A stub that
answers without reading its inputs is unit #2's all-zero kernel window in
another costume.

### GetPath — every layer, and the two red tests that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay, 1 case, scenario 1, `ReadSetParameters.f90:331` | 1/1 `IDENTICAL` over `CHARACTER(1024)` | no-op stub → `OUT_TOL`; **constant stub → PASSES 1/1** |
| differential harness vs clean Fortran | 236 checked, 0 failed | no-op stub → 234/236 failed, naming `PathName` |
| mutation score | 25/25 behavioural killed, 1.000, 0 declared equivalent, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 236 checked, 0 failed | `LEN(PathName) - 1` in the CALL → 199/236 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved, twice** |

The signature crosses whole and `vit interface` was read attribute by attribute
before any C++ was written: `CHARACTER(*) INTENT(IN)` and `CHARACTER(*)
INTENT(OUT)` each become `CHARACTER(KIND=C_CHAR) :: x(*)` plus
`INTEGER(C_INT), VALUE :: len_x`, with the wrapper declaring both dummies exactly
as the original does. `plan.json`'s `bridge_feasible` said `unknown`, from the
pre-`integrates` conformance matrix; it is now `yes`, from the generator that
ships.

The no-op red test fails **234 of 236**, not all 236, and the two survivors are
the `INTENT(OUT)` copy-in defect below rather than slack in the harness.

`PathSep` is `CHARACTER(1), PARAMETER :: PathSep = '/'` from
`SysFiles/SysGnuLinux.f90` — confirmed from the build log's `Setting system file
as:` line, not read off `CMakeLists.txt`. The forward-slash SEARCH takes the
literal `'/'` and not `PathSep`, because in the reference they are two different
things and only the fallback uses the platform parameter (P7).

## Open

- **THE SIMULATION HARNESS DISCARDS `avrSWAP(37)`, AND IT IS THE ONE INPUT IN
  THIS CORPUS AIMED AT `wrap_180`'s BRANCHES.** Unit #27.
  `Examples/vit_sim.py` writes `avrSWAP[36] = 350°` in scenarios 7 and 27 under
  the comment *"Inject avrSWAP values not handled by call_controller"*, and
  `rosco/toolbox/control_interface.py:211` overwrites it from
  `turbine_state['Yaw_fromNorth']` before `call_discon`. Index 23 is injected
  under the same comment and *also* overwritten — it survives only because
  `Y_MeasErr` happens to carry the same value two lines above. Refuted from
  committed `.dbg` channels, no run needed: a heading of 350 would force
  `Yaw_Err` into `[-3.3, +34.0]` and 10,632 of 23,999 timesteps sit outside it
  (`evidence/wrap_180/heading_injection_discarded.{py,txt}`).

  **NOT FIXED, on purpose.** Repairing it changes what the 27 scenarios feed the
  controller, which moves every baseline and every committed `compared` count —
  X3, and the Driver's call. The consequence is priced: with it, `wrap_180`'s two
  branches would be gate-visible and kernel-visible; without it, they are reached
  only by the differential harness. Worth asking of any unit whose inputs come
  from an injected `avrSWAP` index: **grep
  `rosco/toolbox/control_interface.py` for that index before believing the
  scenario's comment.**

- **TWO RED TESTS WITH THE SAME FAILURE COUNT ON THE SAME CORPUS ARE NOT ONE
  MEASUREMENT, AND THERE IS NO CHECK FOR IT.** Unit #27, found by getting it
  wrong: `harness/wrap_180.redtest.json` and
  `harness/wrap_180.postintegration.redtest.json` both report **130 of 136** and
  are blind to **different six cases, overlapping in two**
  (`evidence/wrap_180/the_six_insensitive_cases.{py,txt}`). The wrong explanation
  is left standing in `ad9f755`'s commit message beside the correction (C12).

  Unit #26's `redtest_corpus_skew.py` catches the *across-corpora* form of this
  and reports `0 SKEWED` here, correctly — both runs are 136 of 136. The
  same-corpus form is invisible to it and to everything else, because a red-test
  artifact records a COUNT and not the set of cases that failed. **A candidate
  addition, not taken here:** have the harness emit the failing case INDICES into
  the artifact, at which point two red tests can be compared as sets for free.
  That changes the artifact schema every scored unit writes (X3), so it is the
  Driver's call.

- **THE GATE-VISIBILITY LIST CARRIES ONE NUMBER PER UNIT AND AT LEAST THREE UNITS
  NEED TWO.** Unit #27. `wrap_180` is on the list at 206,976 *and* its
  branch-deleting stub moves 0 of 5,252,000 — the gate can see the unit and
  certifies a translation missing half of it. `saturate`'s upper clamp
  (62 of 62 kernel-invisible) and `sigma`'s two clamps (0 of 5,252,000 each) are
  the same shape. The list answers *can the gate see this unit at all*; nothing
  in this file answers *which of its branches*, and three units now have that
  second number sitting in their evidence. Raised rather than restructured: the
  recount instruction above says read the artifacts, and the artifacts would
  support a second column.

- **`gate/StateMachine.redtest.json` holds 36,577; this file quoted 1,526,538
  for it from unit #20 until unit #26 recounted.** The artifact's perturbation is
  narrow (`VS_State_Region_1_5` → `VS_State_Region_2`, one replacement), not the
  whole-unit no-op the 1,526,538 was attributed to. Nothing about unit #20's
  disposition turns on it — it is gate-visible under either number — so this is
  a provenance question, not a correctness one: **which run produced 1,526,538,
  and was the committed artifact a deliberate supersession or an overwrite?**
  Left for the Driver or for unit #20's next dispatch rather than settled by unit
  #26, which has no standing over another unit's evidence. Raised because the
  recount that found it is the only reason anybody would.

- **SIX of the 21 comparable units have a pre-integration red test taken against
  a different corpus from the green it certifies.** Measured, not suspected:
  `evidence/unwrap/redtest_corpus_skew.{py,txt}`, which reads `checked` out of
  every committed `harness/<U>.json` and `harness/<U>.redtest.json` pair.

  ```
  NotchFilter    green 2652  red test 1380   (-1272)
  SecLPFilter    green 2884  red test 2284   ( -600)
  Int2LStr       green  144  red test  103   (  -41)
  interp1d       green  497  red test  473   (  -24)
  HPFilter       green  832  red test  829   (   -3)
  StateMachine   green 2890  red test 3610   ( +720)   <- the other direction
  ```

  Five more are **not comparable at all** — AddToList, ColemanTransform,
  ColemanTransformInverse, Conv2UC and Read_OL_Input have no pre-integration red
  test to compare — and that is a different answer from "equal", so it is
  reported separately rather than folded in.

  Two things this does NOT say. It does not say any of those units is wrong:
  `NotchFilter`'s red test moved 551,278 gate values whatever corpus it drew, and
  a red test on a *subset* still demonstrates the green can fail. What it says is
  that the pair no longer certifies what it appears to certify, and that
  `StateMachine`'s runs the other way — its red test saw 720 cases its green
  never did, and its own post-integration green is 3610, so the **pre**-green is
  the stale artifact there.

  And the post-integration layer is **26 of 26 clean**, which is structural
  rather than lucky: post mode reuses the generating run's case file, so the pair
  cannot drift. The defect lives exactly where the corpus is regenerated.

  Not acted on here. Re-taking six units' red tests changes what closes a unit
  and touches five other units' evidence; DECISIONS.md raises the mechanical rule
  (a red test certifies a green only if their `checked` counts match) as a
  candidate method amendment for the Driver.

- **The differential harness's CHARACTER-ARRAY refusal is CLOSED, and the way it
  was found is the standing warning.** `vit interface` shipped a working bridge
  for `Words(NumWords)` while the harness generator refused the same declaration
  — so `plan.json` said the signature crossed, the integration worked, and the
  unit's ONLY OUTPUT was invisible to P11 and P12. Two generators over one
  declaration, disagreeing, and only running both showed it. Built in the loop
  repo (`6d13949`). `FileLines(:)` is still refused: assumed-shape, so its
  element count is in a descriptor `build_c_params` does not emit. Rank ≥ 2 also
  still refused. **`FindLine`, `ParseInAry_Opt`, `ParseDbAry_Opt` and the four
  `ParseInput_*_Opt` units all take a `FileLines(:)`, so that is the next
  instrument gap on the critical path** — the refusal that remains is narrower
  than the one that was there, and it is the one that blocks the most units.
- **A revision stamp read from a hand-written pin file was mis-attributing the
  campaign's own evidence, in three places.** CLOSED in the two loop scripts
  (`5b40e1c`, reading `.git/HEAD` before the pin) and in this campaign's
  `scripts/_harness_stamp.py` (which OVERWROTE the correct read with the pin,
  and now fills a missing key only). Kept open as a record of what the stale
  stamps NAME, because the artifacts are not restamped: **unit #7's
  `harness/GetRoot*.json` and `mutation/GetRoot.json` say `loop_rev:
  99b57ab-pinned` and the tree was at `0e92a72`** — the commit carrying that
  unit's own corpus fix — and **every artifact from unit #5 onward says
  `vit_rev: 8c34ceb-pinned` against a VIT checkout at `87a3847`**. The numbers in
  those artifacts stand; the instrument they name is one or more commits behind
  the instrument that produced them.
- **A mutation score can FLAP between runs of the identical command.** Unit #8:
  0.983, 1.000, 0.983 over the same translation and the same 1370 cases, because
  the deciding mutant differs from the original only past the end of a buffer.
  The scoring defect is closed (a declaration is now about the MUTANT, not about
  one run) but the general hazard is not: **a mutation score is not reproducible
  wherever a mutant's only observable difference is undefined behaviour, and the
  run that reads 1.000 is the one that measured least.** Any unit whose score was
  taken once, with survivors of that shape, has a number nobody has repeated.
- **Both of `GetWords`'s extents are constant in all 27 scenarios**, and this is
  P9 at EXTENT granularity — a shape the verification ledger (E5.2) has no column
  for. `len_Line` is `MaxLineLength` = 2048 and `len_Words` is `MaxParamLength` =
  200 at every one of the eleven call sites; `NumWords` is 2 or `AryLen+1`. The
  gate's red test PASSES for this unit and still constrains nothing about what
  happens at another width. Only the differential harness varies them.

- **`vit check -f <file>` MISATTRIBUTES, and at unit #7 it misattributed a
  finding that LOOKED like it belonged to the unit.** FOURTH sighting, at unit
  #8: two findings against a 190-line translation, `narrowing-local` on
  `ExpUCVarName` (clean 1051) and `delimiter-set` on `':/'` (clean 1236, inside
  `GetPath`), against a `GetWords` occupying clean 1150–1216. Third sighting
  below. The two
  findings against `GetRoot` are `narrowing-local` on `ExpUCVarName` (clean 1051,
  inside `FindLine`) and `delimiter-set` on `':/'` (clean 1332, inside
  `PathIsRelative`) — neither in `GetRoot`'s range 1253–1306. The new part is that
  `GetRoot` **does** contain a delimiter set, `'\/'`, so the reported finding is
  a near-miss a reader could plausibly accept. Re-attribute by line range, not by
  plausibility. Fix belongs in VIT: scope the cross-source checks to the
  function, not the file.
- **The differential harness generator has been blind three ways at once, and
  the verdict never said so.** Closed in the loop repo (`0e92a72`) — see the unit
  #7 block above — but kept open here as a standing warning about the SHAPE:
  a harness green is a claim about the cases that were generated, and only the
  mutation survivors say which cases those were. Every one of the three gaps was
  found by asking why a mutant lived, never by reading a `checked N failed 0`.
- **The post-integration harness links PREBUILT Fortran objects.** A red test
  that edits a wrapper and re-runs `harness.sh --post-integration` without
  rebuilding the controller measures the OLD wrapper and reports green — which
  reads exactly like a harness that cannot fail. Measured at unit #7: the same
  perturbation goes 0/726 without a rebuild and 596/726 with one. `harness.sh`
  could rebuild, or refuse to run when a source it links is newer than the object.
- **`vit interface` EMITS A COPY-IN FOR AN `INTENT(OUT)` ARGUMENT.** New at unit
  #6, seen again at unit #7 (26 of 726 no-op cases survive because of it) and at
  unit #8 (27 of 1370, on a CHARACTER ARRAY, where the wrapper reads every one of
  `len_Words * NumWords` undefined bytes before the call). The
  shipped `GetPath` wrapper reads `PathName` — undefined on entry, by
  definition of `INTENT(OUT)` — into the C_CHAR staging array before the call. It
  does not change this unit's answer, because the translation writes every one of
  `len_PathName` bytes. It changes what the instruments can SEE: it is exactly
  why a no-op stub survives 2 of 236 differential cases (a no-op hands back the
  caller's bytes, and twice those already equalled the answer), and it would mask
  a translation that failed to write the tail. Left in the generated wrapper with
  the finding recorded rather than hand-edited — a hand-edit would make the
  shipped bridge disagree with the generator for every unit after it. Fix belongs
  in VIT (X2). **20 of this campaign's 69 units take a CHARACTER dummy**, so this
  will recur.
- **The done-condition CRASHED rather than answering, on a unit whose evidence
  was complete.** `loop/gitrepo.py`'s `file_at` ran `git show` under `text=True`,
  and `_resolves` (K3/P6) calls it only to ask WHETHER an evidence reference
  names a committed artifact — it never reads the bytes. Unit #6 is the first to
  commit a BINARY evidence artifact (the captured KGen state file), so it raised
  `UnicodeDecodeError` out of `subprocess.communicate` and produced no verdict
  and no predicate list. A verifier that cannot report is worse than one that
  reports FAIL: a session cannot tell "the unit is not finished" from "the check
  is broken", and the obvious next move — drop the artifact from the evidence
  list — would have made the evidence WEAKER to satisfy a defect in the thing
  measuring it. Fixed in the loop repo (`74742bc`), verified in both directions.
  CLOSED; kept here because it is the second time an instrument's own failure
  looked like a finding about a unit.
- **The campaign now has FIVE shapes of P9 and the verification ledger (E5.2)
  needs a column for each.** Unit #4 asked for a "hot but cancelled" column;
  unit #6 adds "produced but never consumed"; unit #7 adds "consumed by a live
  line, into a side effect the instrument does not read". All five are consumer-
  or input-space properties that `coverage/line_coverage.json` cannot express,
  because coverage counts entries to a line and every one of these lines is
  entered.
- **A unit can be the IDENTITY on the whole exercised domain, and then the gate's
  standard no-op perturbation is not a wrong implementation.** New at unit #7.
  `GetRoot` strips a file extension and every scenario hands it a name with no
  `'.'`, so all 444,000 calls fall through to `RootName = GivenFil`. A no-op
  returns the caller's own bytes, which through the aliased call site IS the
  right answer. The red test has to perturb the unit to a WRONG value. The same
  property makes the KERNEL a mirror rather than a comparison, so the kernel's
  liveness test also has to be a wrong-constant stub rather than a no-op.
- **A unit's kernel can be STRUCTURALLY one case, and no configuration reaches
  it.** `GetPath`'s call site runs once per process. Unit #2's answer to a
  vacuous window — widen it — does not apply, and `vit.yaml`'s `invocation` is
  irrelevant for any unit called once during initialisation. The check that
  works is the constant stub: a translation that reads NO input and writes the
  captured answer. If it passes, the kernel is a lookup table. This belongs
  beside the existing all-zero-window recipe in RUNBOOK, and it is there.
- **Unit #4's kernel case count is one more than its own window specifies.**
  Re-extracting `Conv2UC` with the identical command yields **62**, both with and
  without unit #6's KGen change; `evidence/Conv2UC/kernel-window-1.statefiles.lst`
  has 63, the extra being `Conv2UC.0.0.21` — outside the configured
  `0:0:1-20`. 20 + 21 + 21 = 62. An extra IDENTICAL case does not weaken that
  unit's 63/63, so nothing about its conclusion changes, but a future run
  comparing case counts across passes should not treat them as stable. Likeliest
  cause is a stale state file from an earlier attempt being swept up.
- **ESCALATION 3 IS CLOSED.** All three items are done and the unit is
  `integrated`. (1) `CHARACTER(:), ALLOCATABLE` crosses a view struct (VIT
  `a2e2c30`) and now a decomposed differential bridge too (`8c34ceb`) — the
  37-of-69 blocker, off the critical path in both generators. (2)
  `TYPE(ExtDLL_Type)` crosses as a PRODUCTION BRIDGE that owns the SAVE
  variable (`rosco/controller/src/vit_extcontroller_dll.f90`) rather than as a
  view: it calls `LoadDynamicLib` (X1) and hands back `ProcAddr(1)` as a C
  function pointer, which is what `C_F_PROCPOINTER` compiles to. (3) The oracle
  exists and changes no verification default.
- **NEW, AND IT IS THE CAMPAIGN'S PROBLEM: `vit integrate --auto-allocate` does
  not work on this codebase.** Three defects, measured, artifact at
  `evidence/ExtController/vit_defects/integrate_auto_allocate.wrapper.f90`:
  its copy-call scan matches ANY `CALL x(..., arg%field, ...)` and hoisted BOTH
  `LoadDynamicLib` and the external DLL call into the wrapper — which would
  call each a second time; it declines a size expression that is a local
  PARAMETER; and its error handling emits `SetErrStat` and
  `CHARACTER(ErrMsgLen)`, OpenFAST names that exist nowhere in ROSCO. This unit
  needs exactly one ALLOCATE, and it is in the committed wrapper with the reason
  beside it. **The next unit that allocates a field of an argument will hit the
  same wall, and hand-editing a generated wrapper does not scale.** Fix belongs
  in VIT (X2).
- **`vit integrate` wrote `verification: simulation` into `vit.yaml`**, having
  run no simulation and read no result. It is FALSE for this unit — the gate is
  blind to it and the committed red test says so. Removed by hand with the
  reason in the file. Same shape as the `// After verification: <name> kernel
  PASSED` comment `37f8bdf` deleted from every generated file: nothing in the
  toolchain checks a claim a generator makes about verification.
- **`vit analyze-types --fix character` must not be used to close this gap.**
  It is what VIT's own error message suggests, and it rewrites ROSCO's type
  definition to a fixed length — changing `LEN(ErrVar%ErrMsg)`, which sizes
  `avcMSG`. That is a behavioural change to the oracle (P7). Recorded here
  because the suggestion is in the tool's output and will be read again.
- **A gitignored file can reconfigure the gate where no clean-tree check can
  see it.** `Examples/DISCON.IN` is ignored at `.gitignore:78`. A run that dies
  on a signal cannot restore it from a `finally`, and `git status` stays clean,
  so `done.py`'s P2 cannot catch it. `gate.py` already snapshots and restores
  these; nothing else did. The general form: **any restore that must survive a
  crash belongs in a parent process, not in the process that crashes.**
- **`coverage/line_coverage.json` cannot distinguish "never ran" from "never
  instrumented".** `scripts/coverage.py` stores only lines with a NON-ZERO hit
  count, so both are the same empty dictionary. Four files read as empty today —
  `ExtControl.f90`, `Constants.f90`, `ROSCO_Types.f90`, `ZeroMQInterface.f90` —
  and only re-running with the denominator printed tells them apart
  (`ExtControl.f90` is genuinely `0/28`, 27 times). The per-scenario log line
  already prints `N/M`; the *artifact* drops M. Storing the denominator would
  close it, and would be an addition.
- **A hot line is not an observable line, and nothing in the campaign's coverage
  data can say which is which.** `Conv2UC` runs 1.3M times and no gate
  perturbation of it moves any output, because its result is only ever compared
  against another of its own results. The verification ledger (E5.2) needs a
  column for this that is distinct from both "unexercised" and "argument held
  constant" — it is a *consumer* property, not a producer one, and neither
  coverage nor the gate's own red test in its usual form can detect it. The
  general instrument found here is to perturb toward ABSENCE (`if (false && …)`)
  rather than toward a different answer; see DECISIONS.md for the candidate
  method amendment this suggests, which is flagged and not made.
- **The differential harness refuses a CHARACTER ARRAY dummy.** PARTLY CLOSED at
  unit #8 and moved to the top of this list. An EXPLICIT-SHAPE one whose element
  count is another dummy — `Words(NumWords)` — now crosses. An ASSUMED-SHAPE one
  — `FileLines(:)`, which is what `FindLine`, `ParseInAry_Opt`, `ParseDbAry_Opt`
  and the four `ParseInput_*_Opt` units take — still does not: its extent lives
  in a descriptor `build_c_params` does not emit.
- **The harness varies a string's length over {1, 3, 8}, not over the 200
  `MaxParamLength` gives it.** `_extent_plan` assigns a lone free extent the
  single value 3; the string stage adds 1 and 8 without touching that plan,
  deliberately, because widening `_extent_plan` would change the case set of
  every unit already scored. A defect that only appears past the eighth
  character is not covered.
- **`vit check -f <file>` attributes findings to the FILE, not the function.**
  It reported `minval-endpoints` and `array-section-row` against a 6-line
  translation containing neither; both came from `interp1d`/`interp2d` elsewhere
  in `Functions.f90`. `--function` only sets the report header. Every finding on
  a multi-procedure source has to be re-attributed by hand, and a checker that
  cries wolf twice per unit is a checker sessions will learn to skip. Fix belongs
  in VIT (X2), not here.
- **One argument of unit #3 is invisible to both bit-exact layers.**
  `aziOffset` is 0 at every `ColemanTransformInverse` call site in every
  scenario. The differential harness covers it and nothing else does. This is P9
  at ARGUMENT granularity — coverage counts a line as exercised while one of its
  inputs is a constant throughout — and the campaign's verification ledger (E5.2)
  should be able to say which arguments each unit's bit-exact layers bound.
- **ESCALATION 1 is ANSWERED: VIT learned the C descriptor.** `37f8bdf` +
  `cf885e3`, reaching all three generators the escalation said it would have to
  — `interface_gen`, `test_validate.generate_fortran_bridge`, and the loop's
  `vitbridge`/`emit`. `AddToList` is integrated on it with a mutation score of
  1.000. **It does not automatically unblock the other three** —
  `Read_OL_Input`, `ParseInAry_Opt`, `ParseDbAry_Opt` — whose `bridge_feasible`
  verdicts came from the same pre-`integrates` matrix and have to be
  re-measured rather than inferred from this one.
- **ESCALATION 2 STANDS, and is sharper.** `AddToList` closed without the gate
  ever seeing it, because the harness and the mutation score do not need it.
  That is the shape of an answer for a dead unit — but only for one whose
  signature crosses. A dead unit that cannot cross has no harness, no mutation
  score, and a vacuous P9, and `loop/done.py` still has no branch for
  `integrated_unexercised`. The vocabulary has the word; the verifier does not.
- **A translation's mutation score can be raised by REMOVING restatements, and
  that is not gaming it.** Two survivors here were a `malloc(isize+1)` and a
  `memcpy(..., isize+1)` restating a quantity the allocation already fixed;
  perturbing either produced a memory error no value comparison can see. Naming
  it once (`nsize`) left one site, and that site decides the extent the caller
  sees. Two others survive genuinely and are DECLARED equivalent with reasons
  (`mutation/AddToList.equivalences.json`). The distinction — removed vs
  declared — is the thing to preserve: declaring the first pair away would have
  recorded a blindness as a property of the mutants.
- **`harness/cppmutate.py` reads a C++ template bracket as a comparison.**
  `static_cast<int*>` mutates to `static_cast<=int*>`, which does not compile. 8
  of 33 mutants on `AddToList` (24%) were that, one point under the 25% at which
  `vit_mutate.py` REFUSES to score. **Seen a second time at unit #4**: 4 of 18
  (22%) on `Conv2UC`'s two `static_cast<unsigned char>`s. Two units, both within
  three points of the refusal threshold, for a reason with nothing to do with
  either translation. This is now a pattern rather than an anecdote and belongs
  in the loop repo's mutator: mask template argument lists the way `cppmutate`
  already masks string and character literals.
- **The differential harness varies this unit's allocation status but not its
  length.** With a single extent parameter `_extent_plan` gives the same value
  (3) in every case, so a defect that only appears at another length is not
  covered by the 43 cases. Recorded in `plan.json`'s `observability`.
- **`bridge_feasible` verdicts in `plan.json` were derived from a column that
  measures the wrong generator.** VIT's conformance matrix fed `bridge` and
  `compiles` from `test_validate.generate_fortran_bridge` — the differential
  harness's Fortran side — while `vit integrate` ships the output of
  `interface_gen`. Fixed in VIT (an `integrates` column, and a refusal), but
  **the plan's 64 `yes` verdicts were computed before that**, and one of them —
  `c_complex_in`'s shape — is now known not to integrate. The plan's
  feasibility column should be re-derived against the new matrix before it is
  trusted again.
- **THREE SCENARIOS EXECUTE NO CONTROLLER CODE AT ALL.** Scenarios 10, 14 and
  24 run 0 lines of Controllers.f90, ControllerBlocks.f90, Filters.f90 and
  Functions.f90; scenario 13 runs 12 lines of Functions.f90 and 0 of
  Controllers.f90. They enter DISCON, read parameters, and stop —
  DISCON.F90 42%, ReadSetParameters 39%, everything downstream 0%. Scenario 14
  reports `PASSED` and 11 of its 13 channels are a single constant. They
  contribute their values to the gate's 5,252,000 while constraining nothing
  about the controller. This is **E3.3's exact failure mode** and E3.3 is still
  `manual`. Measured, not inferred: `coverage/line_coverage.json`.
- **E2.3 is not closed by `coverage/line_coverage.json`.** That file is real
  per-scenario line coverage and it is what C2 now selects call sites from, but
  it comes from a `--coverage -O0` build, not the campaign's Release build. The
  criterion asks for coverage from a clean build committed as phase evidence.
- **The launcher's protect glob has a hole.** `rosco/controller/src/*.f90`
  matches 11 files and NOT `DISCON.F90` — the one file `vit extract` modifies
  (strips CRLF, even under `--dry-run`). Add
  `--protect 'rosco/controller/src/*.F90'`.
- **`vit verify` and `vit integrate` rewrite `vit.yaml` and strip every
  comment.** The file is declared `derive` and its provenance lives in those
  comments; three runs deleted it three times. Currently restored by hand after
  each unit. Either VIT should round-trip comments or the provenance should live
  somewhere VIT does not write.
- **`regress.sh --baseline` does not restore `Examples/DISCON*.IN`.** Only
  `gate.py` snapshots and restores them. A baseline regeneration leaves two
  files modified and the tree dirty, which blocks `done.py`'s clean-tree check.
- **The harness Makefile's LIBS can carry `kgen_utils.f90.o`**, an object that
  exists only because an extraction left it in the build tree. A from-scratch
  build directory would not have it and the link would fail.
- `vit.yaml` is declared `derive` with `red_tested = false`. The planned test is
  to drop `assumed_size_arrays.avrSWAP` and confirm extraction breaks.
- Bootstrap otherwise incomplete: `phases.toml` still declares most criteria
  `manual`, which is NOT_EVALUABLE and never a pass.

## Closed

- **`reset_to_clean.sh` used to leave `CMakeLists.txt` integrated**, so "clean"
  meant the pre-integration SOURCE and not the pre-integration BUILD, and the
  differential harness link died on a second definition of its own translation.
  Closed in `6e614af`: the reset strips translated `.cpp` entries from
  CMakeLists and `restore_integrated.sh` puts them back. `harness.sh`'s LIBS
  workaround is kept — it is what makes the artifacts already committed for
  unit #2 reproducible.

- **`ColemanTransform`'s closure was asserted; now it is measured.**
  `scripts/done_check.py` runs the loop's own `DoneVerifier` with exactly the
  configuration `run_campaign.py` builds. At the start of the second pass it
  returned INCOMPLETE 10/13; it now returns COMPLETE 13/13. The failing
  predicate was P12: `vit_mutate.py` wrote `total`/`equivalent` and P12 reads
  `mutants`/`equivalent_declared`, so a genuine 35/35 was invisible to the check
  that requires one — and it failed with the WRONG REASON, "the operator set
  reached nothing in this unit". Fixed in the loop repo (`13265e7`, both
  spellings emitted) and the artifact regenerated by the tool, not hand-edited.
- **Three defects in this campaign's own scripts, each found by a run that
  should have worked.** All three were invisible while the stale artifacts sat
  on disk looking like results. See DECISIONS.md.
  1. `harness.sh` failed on VIT `d07a716`'s per-unit test directory — the skew
     it existed to reconcile had closed.
  2. Its post-integration LIBS extraction passed the literal words `LIBS` and
     `+=` to the linker, because the new VIT's Makefile has a second `LIBS +=`
     assignment its `^LIBS =` grep could not see. It now asks `make`.
  3. It aborted one line before stamping whenever `./test` exited non-zero — so
     the RED artifact, the one that most needs to say what it measured, was the
     only one that could not.

- **`AddToList` was `blocked`; it is now `integrated`.** The blockage was
  escalation 1, and the answer was to build the feature the refusal was standing
  in for rather than to close the unit as impossible. What made it closable is
  that its verification never needed the gate: a differential harness against
  the clean Fortran (43 cases, both branches red-tested), a mutation score of
  1.000, and a post-integration harness for the wrapper. The gate's green for it
  is still vacuous and is still committed beside the red test that says so.

- **The extraction blocker was a dead call site, not a tool defect.**
  `vit extract` printed `✓ Extraction successful` and `WARNING: No state data
  captured.` for `AddToList` three times. It was aimed at
  `ROSCO_IO.f90:1126`, which coverage now shows has **zero hits in all 27
  scenarios** — as do all five `AddToList` call sites. Pointed at a call site
  that executes, extraction works: `ColemanTransform` at `Controllers.f90:510`
  captured 63 state files, and kernel replay is available as a verification
  route. The tool was reporting success about instrumentation it had genuinely
  installed; the run never reached the pragma.
- **E1.2, previously "declared but unmet".** Closed 2026-08-10 — see Evidenced.
- **Reset-to-clean**, which did not exist: `scripts/reset_to_clean.sh` and
  `scripts/restore_integrated.sh`, red-tested.

## Unit #5, three dispatches — 2026-08-11

**Each dispatch's blocking claim was refuted by the next one, and saying so is
part of the result.**

* **First dispatch** closed `blocked` on *"there is no runnable oracle for
  ExtController anywhere in this campaign"*. The SIGSEGV it measured was real
  and is still true of the campaign's own inputs; the generalisation was not.
  `DLL_FileName` is the literal string `"unused"` in all 14 `Examples/*.IN`, so
  the crash was a property of the INPUT. **Sixty lines of C** made the original
  run to completion.
* **Second dispatch** built that fixture, closed the `CHARACTER(:)` view-struct
  gap and the 132-column bridge-generator gap, and closed `blocked` again on
  three remaining items — one of which it correctly called *a judgement, not a
  feature*.
* **Third dispatch** did the three items and found a fourth that was larger
  than all of them. The judgement (C) is made and written where it can be
  reviewed: `harness/ranges.toml`, four pins, each with the measurement that
  forces it. Every one exists because the REFERENCE crashes on the rest of the
  domain — `ExtController` never checks `ErrVar%ErrStat` after
  `LoadDynamicLib` and calls through a null `ProcAddr` — which is itself a
  finding about upstream ROSCO, and the file says so.

**What the three dispatches cost, and what it bought.** Two `blocked`
dispositions on a unit that closes `integrated`. What made the difference each
time was not new information about the function: it was asking *which
instrument needs this* instead of *is this unit verifiable*. The first dispatch
escalated "an oracle must be constructed" as SPEC §8.4's call because it assumed
an oracle meant a gate scenario; it did not. The third found that the
differential bridge had been discarding outputs for the whole campaign, which
no amount of reasoning about `ExtController` would have surfaced — only
running the harness did.

## Done-condition, unit #5

Run at the end of the third dispatch, after both commits, per RUNBOOK. The
verdict and its transcript are in `evidence/ExtController/done_check.txt`; the
second dispatch's INCOMPLETE 11-of-13 is kept beside it as
`done_check.second_dispatch.txt`, because a verdict that was correct when it was
taken is worth more than a deletion.

Worth carrying, from the second dispatch: the first attempt piped the run into
that file, and **creating the file made the tree dirty**, so the artifact
recorded `P2 FAIL dirty_tree` — describing itself rather than the unit. A
done-condition capture has to be taken before the file that captures it exists,
or written as a transcript.

## Unit #29 `CheckInputs` — the mutation re-take (third dispatch)

The one thing left was a valid mutation run. `mutation/CheckInputs.json` carried
`not_evaluable: true` and `compared_against: "THE MUTANT ITSELF -- INVALID"`,
and `vit_mutate` refuses that configuration outright now. Re-taken on the clean
tree:

```
192 mutants   19 nocompile   173 behavioural   8 killed   165 survived   0.0462
reference side, from nm:   ReadSetParameters.f90.o defines
                           __readsetparameters_MOD_checkinputs, no checkinputs_c
```

**The prediction in `bridge_feasible` is CONFIRMED and nothing in this dispatch
disturbs it.** The signature crosses: three view types (`LocalVariables`,
`ControlParameters`, `ErrorVariables`) populated and passed by `C_LOC`, an
assumed-size `REAL(ReKi)` array, a scalar `INTEGER`, all three views
reverse-copied back. `--reverse-copy` is required because the unit's only
outputs are `ErrVar%aviFAIL` and `ErrVar%ErrMsg`, both inside a view. That
wrapper is what the harness's Fortran side calls in the post-integration run
(16,769 / 0) and what the gate exercised over 27 scenarios — and it is also
exactly why the mutation run had to leave the integrated tree, since a wrapper
that works is a wrapper that routes the reference call back into the mutant.

**Two findings, and the first is a correction to this campaign's own RUNBOOK.**

1. The invalid run scored 0.0231 and the valid one scores 0.0462. The target
   layer said a clean-tree re-take gives "a number at the other end of the
   range"; it does not. The tell that found the defect — 169 survivors on a
   corpus that passes 16,769 of 16,769 — is a true observation that does not
   discriminate, because 165 survivors is what this unit produces when the
   instrument is working. `nm` on the reference object is what settles it.
2. Why 165 survive, measured on the clean tree with three one-line
   perturbations of the single message sink: a no-op `ErrMsg` fails 16,769 of
   16,769 (so every case raises an error and `aviFAIL` is `-1` in all of them)
   and a first-writer-wins `ErrMsg` also fails 16,769 of 16,769 (so the first
   failing check differs from the last in every case — every case raises at
   least two). `CheckInputs` has no early return, so the last check wins, and
   the single discriminating output of a 180-check validator is whichever check
   happens to be last. Everything above it is invisible.

Killing the remaining 165 needs cases that fail exactly ONE check. No rule in
`harness/` generates one, and writing it would move every unit's corpus (X3), so
it is named here and not attempted. The survivors are **not** declared
equivalent: `negate_cond` on `(LoggingLevel < 0) || (LoggingLevel > 3)` inverts a
real check and survives because nothing can see it.

**Procedure.** The sweep is 32 minutes against a 600-second foreground command,
so it ran as five `--operator` invocations, each blocking, unioned by
`scripts/_mutation_merge.py` — which refuses a union that does not cover the
sweep, taking the operator population from `harness.cppmutate` rather than from
the parts. The reset window was opened twice and closed twice, with a commit
before each opening and immediately after each closing; nothing was backgrounded
and nothing polled.

## Unit #31 `Debug`, second dispatch — 0.6798 → 0.9226, and one real defect

`deferred` again, on P12, and the number is a different number:
**143 of 155 killed, 0.9226**, 181 behavioural mutants, 26 declared equivalent,
twelve alive and each classified. Raw, before any declaration, **143 of 181 =
0.7901** against the first dispatch's 121 of 178 = 0.6798.

**A REAL DEFECT WAS FOUND, AND ONLY THE NEW STREAM COULD SEE IT** (C12,
`0dbf443` → `106d170`). The unit's `WRITE(*,100)` status line goes to unit 6,
and nothing in this campaign had ever compared that stream. It is comparable and
it was RED: `libgfortran` emits a preconnected unit's record whole, while a
fully-buffered `stdout` split one record mid-field and delivered eighteen more
after the driver's own output. 46 of scenario 27's 110 stdout records differed.
The fix is one `std::fflush`; negating its guard now dies on 283 records, so the
fix is under measurement rather than taken on trust.

**What moved the score, in order of size.** Nothing was argued away first — the
kills came first and the declarations after.

    the stdout stream                    9 kills
    scenario 31  modes off, padded name  5
    scenario 32  two control groups      3
    scenario 33  synthetic drive at LL=3 2
    scenario 30  LoggingLevel = 2        1
    the cppmutate repair                 2
    26 equivalence declarations          0.7901 -> 0.9226

**Six scenarios were added, and the two that earned NOTHING are the better
findings.** Scenario 29 (`LoggingLevel = 0`) cannot reach the unit at all —
`DISCON.F90:145` guards the call site with the same predicate the mutant
attacks, so the inner guard is dominated by the caller's. Scenario 34 (two
Inits) cannot reach `avrIndices`' deallocate arm — a second Init in one library
load is refused before `Debug` is called, and a second after `kill_discon`
reloads the library. Both survivors became equivalence declarations resting on a
measurement.

**The twelve survivors, classified.** None is (a).

    (b) 2   Fl_PitCom and NacVaneOffset are zero in all eight scenarios --
            NOW MEASURED: scenario 35 kills both, on 3,998 and 3,999 records.
            Not folded into the score because a corpus change invalidates
            every part already taken; folding it in gives 145/155 = 0.9355
    (b) 3   a trim that never runs to the start of the name, and a close guard
            that differs only when the open FAILED
    (c) 6   a difference needing a computed double exactly on 1E-99 / 1E+99 --
            no admissible input selects a debug channel's value, and for
            LocalVarOutData the source domain is a 4-byte float
    (c) 1   an out-of-bounds write into allocator padding

The two (c) classes name their instruments — a direct driver over a synthetic
`DebugVariables`, and a sanitiser build — and neither is built here. Both are in
`DECISIONS.md` as proposed amendments, because both change what "killed" means
for every unit rather than for this one.

**Procedure.** Ten foreground sweep parts, each under `mutate_guarded.sh` and
each routed through the clock; nothing backgrounded by hand, nothing polled. The
reset window was opened once and closed in the same command. Nine commits, one
per expensive artifact. `revcheck --unit Debug` is clean at 14 artifacts, all
naming `3f8ed43` — the gate, its red test and the post-integration harness were
re-taken rather than left naming the superseded mutator.

**One gap was closed as a measurement after the sweep.** The two zero debug
channels were predicted to need a scenario above rated. They did not: running
scenario 33's drive at 15 m/s moved neither. `Fl_Kp` and `Y_MErrSet` are
`0.0000` in all 14 shipped `DISCON*.IN`, so the floating feedback is a
proportional law with no gain and the vane offset IS the parameter — the same
shape as `LoggingLevel = 1` hiding a third of this unit, one layer down.
`run_scenario_35` sets both and both mutants die
(`mutation/Debug.ablation-s35.index_offset.1.json`). It is kept OUT of
`mutation/Debug.json` because the merge refuses parts that ran different
scenarios; the next dispatch folds it in and re-sweeps for 145/155 = 0.9355.
