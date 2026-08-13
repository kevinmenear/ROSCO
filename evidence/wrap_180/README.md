# Unit #27 — `wrap_180` — evidence

`REAL(DbKi) FUNCTION wrap_180(x)`, three statements:

```fortran
IF (x .le. -180.0) THEN
    wrap_180 = x + 360.0
ELSEIF (x .gt. 180.0) THEN
    wrap_180 = x - 360.0
ELSE
    wrap_180 = x
ENDIF
```

Disposition: **`integrated`**. Five layers available, five run, all green, all
red-tested. The unit is small and the interesting thing about it is not the
translation — it is that **two of the five layers cannot see either of its
branches, and that is a measured number rather than a caveat**.

| layer | result | red-tested |
|---|---|---|
| kernel replay | 62 of 62, 13,950 field rows all `IDENTICAL` | VIT's own (`x × 1.00001`), plus three stubs — see §2 |
| differential harness | 136 checked, 0 failed, 0 inadmissible | no-op **130 of 136**; both branches deleted **31 of 136**; each branch alone 13 and 18 |
| mutation | **11 of 11 killed, score 1.000**, 0 declared, 0 uncompilable | — (this *is* the red test, eleven times) |
| post-integration harness | 136 checked, 0 failed, against the shipping library | the wrapper hands `-x`: **130 of 136**, revert verified green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | whole unit as a no-op **206,976 of 5,252,000**, revert 0 |

---

## 1. C2 — the branches are dead everywhere, and the reason is a discarded input

Two read-only censuses over committed artifacts, run **before** extracting.

**`coverage_branch_deadness.py`** reads `coverage/line_coverage.json` and puts
three independently-derived counts beside each other:

```
hits at the FUNCTION line               675,987
hits summed over the six CALL SITES     675,987   EQUAL
hits on the ELSE line (pass-through)    675,987   EQUAL
hits on BRANCH 1 (x <= -180 -> +360)          0
hits on BRANCH 2 (x >   180 -> -360)          0
```

Every one of 675,987 calls in all 27 scenarios took the pass-through arm. The
identity is not vacuous and P10's control is in the same file: **`wrap_360`**, two
screens down, is the same shape with a live branch — `x >= 360` carries 15,199
hits in scenario 22 — so a branch body does get its own coverage entry when it
runs, and an absent key means zero rather than un-instrumented.

**`heading_injection_discarded.py`** is the half that a source read gets wrong.
Reading `Examples/vit_sim.py` says scenario 7 drives the yaw path hard:

```python
nac_heading_rad = 350.0 * deg2rad
# Inject avrSWAP values not handled by call_controller
controller_int.avrSWAP[36] = nac_heading_rad   # avrSWAP(37) NacHeading
```

`ReadAvrSWAP` sets `LocalVar%NacHeading = avrSWAP(37) * R2D`, so `x =
NacHeading + NacVane` at `Controllers.f90:400` should sit in **[330, 370]** on
every one of 23,999 timesteps — squarely inside `x .gt. 180`. Coverage says that
branch never runs. Coverage is right:

```
rosco/toolbox/control_interface.py:209   self.avrSWAP[23] = turbine_state["Y_MeasErr"]
rosco/toolbox/control_interface.py:211   self.avrSWAP[36] = turbine_state["Yaw_fromNorth"]
```

`call_controller` re-populates avrSWAP **after** the injection and **before**
`call_discon`. Two of the six indices under that comment *are* handled by
`call_controller`; index 23 survives only by coincidence (`Y_MeasErr` carries the
same `nac_vane_rad`), and index 36 is replaced by the accumulated yaw position,
which starts at 0.

Falsified from the committed `.dbg` without running anything. `Yaw_Err` is
`wrap_180(NacHeadingTarget - NacHeading)` and both operands are channels:

```
NacHeadingTarget observed        [ -13.3032,  +23.9830]
Yaw_Err          observed        [ -19.0082,  +19.0082]
Yaw_Err PREDICTED if heading=350 [  -3.3032,  +33.9830]
timesteps OUTSIDE that interval  10,632 of 23,999
```

**So the corpus's one deliberate attempt to push this unit past ±180 never
reaches the controller.** That is a fact about the harness *around* ROSCO, not
about ROSCO, and it is the reason two of five layers are blind below.

Call site chosen: **`Controllers.f90:400`, scenario 7**. Of the six sites, three
pass `atan2(...) * R2D`, whose range is `[-180, 180]` — at those the high branch
is unreachable *by construction*, not merely by corpus. `:400` is one of the
three whose argument is a plain sum, so it is the site where the branches are
reachable in principle. Enclosing scope is `SUBROUTINE YawRateControl`, which is
unit #24's KGen constraint. Unit #24 says to run the branch-deleting stub at each
candidate site before spending the cycle on one; **here the committed coverage
settles it one step earlier and cheaper** (unit #25's rule), because the branch
bodies have zero hits at *every* site — no site can see them, so no stub run can
distinguish sites on that axis. Said explicitly, as unit #25 requires: the
purpose was served, the literal procedure was not followed, and the choice fell
to which site's *argument domain* admits the branches.

## 2. Kernel replay — alive on every case, blind to the whole point of the unit

`vit verify`: 62/62, **13,950 of 13,950 rows `IDENTICAL`**, read out of
`verify_fields.csv` rather than off the verdict line (unit #16), with no `IN_TOL`
and no NaN (unit #22's hole). `winddir` is among the 225 compared fields.

The captured domain, read from the field log's `reference` column
(`captured_domain.txt`):

```
cases 62      distinct values of x 62      x in [-4.4206384497, +1.5171040094]
x == 0.0 exactly   1 case      winddir == x   62 of 62
```

**A function whose branches turn at ±180, sampled over a window six degrees
wide.** Three stubs, forced-rebuilt and hash-verified inside the container
(`run_stub.sh`, units #18 and #23):

| stub | passed | what it says |
|---|---|---|
| both wrapping branches deleted | **62 of 62**, 14,260/14,260 `IDENTICAL` | the kernel is **blind to both branches** |
| constant `-7.25` (determinate, outside the domain) | 0 of 62 | the kernel is **alive on every case** |
| constant `0.0` | **1 of 62** | unit #25's rule recurring — see below |

The zero stub is kept because it is not a redundant run. It is the campaign's
default liveness stub, and it scores a **pass** here, on the one case whose
captured `x` is `0.0` exactly. Unit #25 learned this on `sigma`'s `y0`; the shape
transfers to any unit whose captured domain contains the stub's constant.
`-7.25` was chosen against the *call site's arguments*, which is why it fails 62
of 62 where zero fails 61.

## 3. Differential harness — the only layer that reaches the branches

136 cases against the clean Fortran, **0 failed, 0 inadmissible**
(`harness/wrap_180.json`). R6 put the two thresholds in the stream as values:

```
PREDICATE KNOB: x at [-181.0, -180.0, -179.0, 0.0, 1.0, 2.0, 179.0, 180.0, 181.0]
```

Both `-180.0` and `180.0` are literals in the unit's own body, so
`literals_from` sees them directly — the easy case of the rule unit #26 had to
reach through `named_constants_from` because its `-PI` lived in another file.

| red test | failed | artifact |
|---|---|---|
| the whole unit as a no-op | 130 of 136 | `harness/wrap_180.redtest.json` |
| **both** wrapping branches deleted | **31 of 136** | `harness.passthrough-stub.json` |
| the `x <= -180` branch deleted | 13 of 136 | `harness.no-low-branch-stub.json` |
| the `x > 180` branch deleted | 18 of 136 | `harness.no-high-branch-stub.json` |

**13 + 18 = 31.** The two branches are disjoint, so deleting them one at a time
must partition the cases that deleting both moves; that identity is the internal
check on the three stubs. All four red tests and the green are the same 136-case
generating run, so the pair cannot skew (unit #26's census, re-run: 0 skewed).

## 4. Mutation — 11 of 11, and the two-case margin counted rather than trusted

`mutation/wrap_180.json`: **11 mutants over 5 operators, 11 killed, 0 survived,
0 declared equivalent, 0 failed to compile — score 1.000.** Scored against the
clean build, before integration.

```
compare_op     '<=' -> '<'                        killed   2/136
compare_op     '>'  -> '>='                       killed   2/136
arith_op       'x + 360.0' -> 'x - 360.0'          killed  13/136
arith_op       'x - 360.0' -> 'x + 360.0'          killed  18/136
swap_operands  'x - 360.0' -> '360.0 - x'          killed  28/136
const_tweak    '180.0' -> '181.0'   (x2)           killed   2/136 each
const_tweak    '360.0' -> '361.0'   (x2)           killed  13, 18/136
negate_cond    'if (x <= -180.0)' -> '!(...)'      killed 120/136
negate_cond    'if (x > 180.0)'   -> '!(...)'      killed 107/136
```

There is no `.equivalences.json` and no `.undeclared.json` for this unit, and the
absence is structural rather than an omission: **the survivor set is empty**, so
an undeclared run is byte-identical to the declared one and there is nothing to
excuse. `mutation/wrap_180.undeclared.json` was written and then deleted for that
reason.

`boundary_margin.py` is about the two thin kills. Each of those `compare_op`
mutants differs from the reference on **exactly one input value**, so `2 of 136`
is not a sample statistic — it is the multiplicity of `-180.0` and `+180.0` in the
case file. Computed both ways, from the `.bin` the harness actually ran:

```
'<=' -> '<'    cases where it differs from the reference   2
               cases holding x == -180.0                  2
               the differing cases are exactly those    True
multiplicity   {180.0: 2, -180.0: 2}
```

R6 emits each boundary twice (the literal ladder and the predicate knob). **Take
that block away and both mutants survive at any corpus size** — the margin is a
corpus rule, not a case count. Same shape as unit #26's `.LE.`/`.LT.` boundary and
unit #24's signed zero: the rule that decides a unit's score was added by an
earlier unit.

## 5. Post-integration and the gate

Integrated with `vit integrate --apply`, **no `--reverse-copy`**: the unit's only
dummy is `REAL(DbKi), INTENT(IN) :: x`, so unit #23's grep test returns 0 — there
is no view-type INOUT argument and no scalar field for a wrapper to drop.

```fortran
FUNCTION wrap_180(x) RESULT(wrap_180_result)
    wrap_180_result = REAL(wrap_180_c(x), 8)
END FUNCTION wrap_180
```

**Post-integration harness: 136 of 136** against the shipping library. Red-tested
*in the wrapper*, which is the only place this layer is the sole witness — with
no reverse-copy line to delete, the perturbation is the argument handoff (`-x`
for `x`), rebuilt between the edit and the run both ways: **130 of 136**, then
revert, rebuild, **136 of 136** again.

**Gate: 5,252,000 values / 351 channels / 27 scenarios, 0 mismatched**, taken
*after* `vit integrate` (unit #23's re-take rule). Both perturbations anchored to
`double wrap_180(double x) {`, asserted to occur exactly once in the tree
(unit #26):

```
the whole unit as a no-op       206,976 of 5,252,000 moved, revert 0
BOTH wrapping branches deleted        0 of 5,252,000 moved, revert 0
```

The first is `gate/wrap_180.redtest.json`, and it doubles as the **same-build
control** for the second: the chain from build to install to 27 simulations to bit
comparison is alive on this libdiscon, so the zero is this unit's branches being
invisible rather than a broken gate. No borrowed control was needed — unlike
`unwrap`, whose own no-op moved nothing and had to re-run GetWords'
perturbation.

**Three instruments on one stub.** Both branches deleted: gate **0 of
5,252,000**, kernel **62 of 62 PASSED**, harness **31 of 136 FAILED**. Two
bit-exact layers over 675,987 real calls say a translation missing half its body
is correct, and a 136-case generated corpus says it is not.

## 6. The correction — two red tests reporting `130 of 136` are not one measurement

Recorded against my own wrong artifact rather than fixed quietly (C12). The commit
message of `ad9f755` explained the matching counts as *"the same reason: 0.0 and
-0.0 map to themselves under negation, and the symmetric rungs of the magnitude
ladder pair up."* That was an argument, and it is wrong twice. The wrong claim
stays in the git log; `the_six_insensitive_cases.py` corrects it:

| perturbation | blind on | why |
|---|---|---|
| no-op (`return 0.0`) | 4 cases at `x = 0.0`, plus `x = ±360.0` | `ref(x)` **is** `0.0` there |
| sign flip (`-x`) | the **four boundary cases** `x = ±180.0`, plus `x = ±360.0` | `ref(x) == ref(-x)` |

Overlap: **two**, not six. And the negation set is the better half: `.le.` on the
low guard and `.gt.` on the high one send **both** endpoints to `+180`, so a sign
flip is unobservable at exactly the four cases that exist to pin this unit's
asymmetry — the property the translation's own comment is about is what hides the
perturbation there. `-0.0` is in **neither** set: `ref(-0.0) = -0.0` moves under
the no-op, and `ref(+0.0) ≠ ref(-0.0)` in bits moves under the sign flip, so the
one mechanism the wrong claim named is the one the corpus rules already close.

General, and raised in DECISIONS.md: **two red tests with the same failure count
on the same corpus are not the same measurement.** Unit #26's census compares
red-test counts *across* corpora and found six skewed pairs; this is the same
hazard with the corpus held fixed, where nothing looks wrong at all.

## 7. Files

| file | what |
|---|---|
| `wrap_180.final.cpp` | the shipped translation |
| `coverage_branch_deadness.{py,txt}` | §1, three counts and the `wrap_360` control |
| `heading_injection_discarded.{py,txt}` | §1, the 350° heading refuted from the committed `.dbg` |
| `captured_domain.txt` | §2, the kernel's 62 inputs, all of them |
| `run_stub.sh` | §2, kernel stub runner (forced rebuild + hash guard) |
| `wrap_180.{passthrough,wrong-constant,zero}-stub.cpp` | §2, the three kernel stubs |
| `kernel.Controllers400.*` | §2, the shipped kernel run, its field log, and the three stub runs |
| `run_harness_stub.sh` | §3, harness stub runner |
| `wrap_180.{noop,no-low-branch,no-high-branch}-stub.cpp` | §3, the harness stubs |
| `harness.{passthrough,no-low-branch,no-high-branch}-stub.json` | §3, 31 / 13 / 18 of 136 |
| `boundary_margin.{py,txt}` | §4, the two-case margin counted in the case file |
| `run_wrapper_redtest.sh` | §5, the wrapper perturbation with rebuilds both ways |
| `harness.postintegration.revert-verified.json` | §5, green after the revert |
| `gate.both-branches-deleted-MOVES-NOTHING.json` | §5, 0 of 5,252,000, with its notes |
| `the_six_insensitive_cases.{py,txt}` | §6, the correction |
| `vit_translate.stdout.txt` | the scaffold prompt, as generated |
| `../../harness/wrap_180{,.redtest,.postintegration,.postintegration.redtest}.json` | §3, §5 |
| `../../mutation/wrap_180.json` | §4, 11 of 11, 1.000 |
| `../../gate/wrap_180{,.redtest}.json` | §5 |
| `done_check.txt` | the done-condition as it stood at the state commit |
