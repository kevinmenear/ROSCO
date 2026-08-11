# DECISIONS — rosco-r2

Append-only record of *why*. Never read end to end.

## 2026-08-10 — Phase 1: the gate got an artifact, and extraction got a verdict

**The gate counts values, not channels.** `regress.sh` compared whole channels
with `np.array_equal` and reported 351. The same comparison at element level is
5,252,000. `loop/done.py` P9 only asserts `compared > 0`, so a channel-counted
artifact would have passed while claiming four orders of magnitude less than it
appeared to — and, worse, would not have been evidence about the instrument
E3.2 observed failing, because the red test in RUNBOOK.md is recorded in values
(198,892 of 624,000). One criterion's artifact describing a different instrument
than the next criterion's evidence is not a bookkeeping problem; it means
neither number constrains the other.

**`scripts/gate.py` exits non-zero when it compares nothing.** Not part of E3.1
as written. E3.1 asks for the count to be printed; SPEC §7 explains that the
count is what catches the vacuous case, and a gate that compares zero values and
exits 0 is the failure this project exists to remove. Recorded here as a local
decision so nobody later reads it as the criterion.

**The gate restores the input files it dirties.** Found by running it: two
`Examples/DISCON*.IN` files came back modified (`IPC_ControlMode` 1→2,
`F_NumNotchFilts` 0→1), because `vit_sim.py`'s `write_discon()` rewrites them
per scenario in place. The loop verifies with `require_clean_tree=True`, so this
would have blocked every unit from closing — and the obvious fix a session would
reach for, `git add -A`, would have committed a silently reconfigured gate for
every unit that followed. An instrument that edits its own subject is not
measuring it. `gate.py` snapshots and restores, names what it restored in
`inputs_restored`, and reports anything still dirty in `residual_dirt` rather
than trusting its own list to be complete.

**`vit.yaml` is derived from bf25e35, not from the first replication's HEAD.**
At HEAD that file carries 54 `status: integrated` entries naming the .cpp files
a translator is meant to produce, two extra `types` entries, a `shared_files`
list and a widened kgen `invocation` — all learned *during* that run. Copying it
would have pre-declared 54 of this campaign's 69 units done and handed over an
answer key. bf25e35 is the first replication's *setup* commit, which knew none of
it, and is the revision `campaigns/rosco-r2.toml` already pins as `src_rev`. A
semantic diff of the two YAMLs shows exactly four keys differing — the mechanical
`/workspace/ROSCO-replication` → `/workspace/ROSCO-r2` path substitution — and
nothing else. Declared `derive`, and `red_tested = false` until the derivation
has actually been observed failing.

The judgement this replaced is worth recording: asked whether `types`,
`allocatable` and `shared_files` were setup or answer key, the honest answer was
not available by reasoning about them. It was available by reading the pinned
revision, which shows `types` with three entries and `allocatable` present at
setup, and `shared_files` absent. Deriving from the revision the manifest
already names beat deciding on their behalf.

**Extraction is recorded as NOT WORKING, against its own success message.**
`vit extract` prints `✓ Extraction successful` and then `WARNING: No state data
captured.` Three runs, zero files matching the campaign's state pattern, while
the first replication has `kernel/AddToList.0.0.1` for the same function. The
instrumented library, the run command and the invocation window were each ruled
out by measurement rather than argument (see RUNBOOK.md). The cause is not yet
known. Recording it as TODO-with-evidence rather than as a working step is the
whole point: a green that was never observed going red is what this campaign is
built to refuse, and here the green is coming from our own front-end.

Consequence for planning: kernel replay is not currently an available
verification route, so units must close on the generated differential harness.
Six of them are `respecify`, for which P13 makes a mutation score mandatory
anyway.

## 2026-08-10 — bridge_feasible was a reading of a parse failure

`AddToList` was unit #1, `contract: mirror`, `bridge_feasible: yes`. Its whole
contract is `allocate(clist(isize+1))` → copy → `move_alloc(clist, list)`: it
changes the extent and the data pointer of the caller's allocatable array. The
bridge VIT generates for it passes `list(*)` as a bare pointer and `n_list` BY
VALUE, and the wrapper's dummy drops `allocatable` altogether. The function's
only effect has no representation in its own signature.

The plan already said so, two entries away. `ParseInAry_Opt` and
`ParseDbAry_Opt` were `bridge_feasible: no` with the basis `c_alloc_inout does
not cross (INTENT(INOUT) ALLOCATABLE needs an ALLOCA)` — the identical feature.

**The mechanism was not the parameter name.** `harness/contract.py`'s `_DECL`
matched `^(CHARACTER|INTEGER|...)\b[^:]*::`, and `[^:]*` forbids a colon
anywhere in the attribute list — so every declaration carrying `DIMENSION(:)`
or `dimension(:,:)` failed to match and the argument silently got no entry in
`arg_decls`. Measured on this tree: 15 of 418 dummy arguments lost, in 12 of 83
procedures. `AddToList`'s `list` was one of them, so `bridge_blockers` iterated
a one-argument list, found no ALLOCATABLE, and returned (). The evidence of `no`
had been parsed away and the absence was rendered as a positive.

All 67 `yes` verdicts carried the same basis string — "no signature feature is
recorded as failing to cross" — which states in words that nothing was
recorded. `Tri` requires a basis on a settled value precisely so a yes cannot be
told from a default; a constant met that requirement syntactically and defeated
it semantically. A constant is not a provenance.

Fixed in the loop repo: the regex now admits colons inside the attribute list;
`_feasibility` returns UNKNOWN if any argument's declaration did not parse,
rather than a verdict about the arguments that happened to be readable; and a
`yes` basis now names the arguments it read. Both fixes are red-tested.

Re-derived, unchanged sources, same entry point and exclusions:

    bridge_feasible   67 yes / 2 no   ->   64 yes / 5 no
    changed:  AddToList     yes -> no   (list: c_alloc_inout)
              Read_OL_Input yes -> no   (channels: c_alloc_inout)
              interp2d      yes -> no   (zdata: c_assumed_shape_2d)

`interp2d` fails on a different cell — a rank-2 assumed-shape dummy — a blocker
class the old plan could not see at all. Nothing else in the plan moved: unit
set, order, contracts, absorption, exclusions and notes are byte-identical, so
this is a contained correction and not a re-planning.

**Consequence not yet addressed.** `loop/driver.py:82`'s `next_unit` selects on
`disposition` alone and ignores `bridge_feasible`, so the Driver would still
dispatch `AddToList` first — now against a plan that states its signature cannot
cross. Whether an infeasible unit should be skipped, escalated, or given a
disposition up front is a policy question, recorded here rather than decided in
passing.

## 2026-08-10 — what "the smoke test passed" means, written before launching

Named in advance, because afterwards any outcome can be narrated into either
column. The smoke test is `--only ColemanTransform --max-units 1`, and what it
tests is the WIRING, not the translation.

  PASS   ColemanTransform closes all 13 predicates.

  PASS   It fails at P11 or P12 because the harness or the mutation score could
         not produce evidence, HAVING REACHED that point through
         translate -> verify -> integrate -> gate. The path working is the
         result. Record it as a finding about the evidence path, not as a
         failure of the loop.

  FAIL   Anything earlier: dispatch, Sentinel, budget accounting, prompt
         render, done-condition evaluation, or the state commit.

**`--mutation-glob` will not be unset to make the unit close.** `done.py:344`
returns `self._mutation(...)` unconditionally when the glob is set — the
red-test fallback exists only when it is UNSET — and `min_mutation_score` is
1.0 and is never overridden by `run_campaign.py`. So mutation is mandatory for
all 69 units at a 100% kill rate, mirror and respecify alike. Dropping the flag
would convert a hard requirement into a red test and make the first unit's
evidence permanently weaker than every later unit's, silently. If mutation
genuinely cannot run here, that is a Phase 1 blocker to fix or escalate, not a
flag to drop.

## 2026-08-10 — the loop repo runs from a pinned checkout inside the mount

`vit_harness.py` and `vit_mutate.py` need three things at once: ROSCO-r2's
Fortran objects, this repo's `harness/` package, and a Linux toolchain. No
environment had all three. `vit-dev` mounts the campaign but not the loop repo;
`vit-harness` mounts the loop repo but not the campaign; the host has both plus
gfortran, but it is macOS and the objects are Linux, so they cannot link.

Both scripts resolve their imports repo-relatively
(`sys.path.insert(0, parents[1])`), so they need the repo PRESENT at any path --
not a new mount and not re-plumbing. So a checkout lives at

    ~/Artifacts/vit_translation/translation-loop   ->  /workspace/translation-loop

cloned from the LOCAL repo, not origin: origin does not have the fixes. No
container was recreated. Compilation now happens in the same container that
built the objects and ran the gate -- identical, not merely equivalent, which
matters because equivalence is a measurement and identity is a property. (The
toolchains do happen to be identical: both images carry gcc/gfortran/g++ 13.3.0
on Ubuntu 24.04 with glibc 2.39, despite being different images.)

**Pinned to `d58a418`, recorded here, and reversible with `rm -rf`.** Two
checkouts drift, and a campaign quietly running month-old harness code is the
silent-wrong-answer shape this project exists to remove. So both scripts now
stamp `loop_rev` into their JSON beside `{against, checked, failed}` and
`{total, killed, equivalent, score}`: the evidence already said what it measured,
this says which instrument measured it.

`vit-dev` has NO git, so the stamp returned "unknown" there -- silently, in the
direction that looks like it worked. It now falls back to a `.loop_rev` pin file
and reports `d58a418-pinned`, never dressed up as a verified read: it states
what the pin claims and cannot see whether the tree was edited after. Live git
still wins where it exists.

Verified against ROSCO-r2 rather than assumed: `vit_harness.py` generated 257
cases for ColemanTransform with the rules applied, and `vit_mutate.py` reached
the point of reading the translation and failed with a clean FileNotFoundError
because no translation exists yet -- which is the smoke test's job to produce.

Separately: `vit-dev`'s image was UNTAGGED (`b83f3bc715d0`). A `docker image
prune` would have deleted the campaign's execution environment, which holds two
weeks of state and is not rebuildable from a Dockerfile we have. Now tagged
`vit-dev-rosco:latest`.

## 2026-08-10 — unit #2: the extraction blocker was a dead call site

`vit extract` was recorded here as NOT WORKING, against its own success message:
three runs on `AddToList`, each printing `✓ Extraction successful` and
`WARNING: No state data captured.` The instrumented library, the run command and
the invocation window had each been ruled out by measurement, and the conclusion
drawn — reasonably, on that evidence — was that the front-end was broken and
kernel replay unavailable.

It was aimed at a line that never runs. `coverage/line_coverage.json` now
measures per-scenario line hits across all 27 scenarios, and
`ROSCO_IO.f90:1126` has **zero** in every one of them. So do the other four
`AddToList` call sites. There was no state to capture; KGen instrumented a
region the process never entered, and reported — correctly — that it had
instrumented it.

Pointed at `Controllers.f90:510`, which runs 23,999 times in scenario 27,
extraction captured 63 state files on the first attempt. Kernel replay is
available.

**What went wrong was the order of the checks, not any of them.** Each thing
ruled out was ruled out properly. What was never measured was the premise every
one of those checks rested on: that the call site executes. The previous entry
even records the suspicion — "the call site is unreachable in the default
scenario. True but not the whole story" — and then moved past it after
re-running against scenario 3, where it is equally dead. A hypothesis discarded
because a *different* scenario also failed is not a hypothesis tested.

## 2026-08-10 — a green kernel that a zero-writing stub also passes

`vit verify ColemanTransform` returned `62/62 passed`, `13,950 field entries`,
every one IDENTICAL. Then the same command, against a translation that reads
none of its arguments and writes `0.0` to both outputs, returned `21/21 passed`,
`4725/4725 IDENTICAL`.

The setup invocation window `0:0:1-20` captures the first 21 calls at the
extracted call site, and at `Controllers.f90:510` all 21 land at simulation
start, where `Azimuth` and `rootMOOPF` are 0. Every case fed the function zero
and expected zero back. The comparison was real, 4,725 values were compared,
and the measurement could not distinguish a correct translation from a constant.

**Widening the window did not fix it, and that is the more useful half.** With
`0:0:2000-2020,0:0:3900-3920` against scenario 6 — which
`coverage/line_coverage.json` shows executing the call site 3,999 times and the
function body 7,998 times — all 62 cases were *still* all-zero. Scenario 6 is a
1-DOF simulation that never drives azimuth or blade root moments. The line
executed 7,998 times on zeros.

That is P9 as a measurement rather than a maxim. Coverage says a line ran. It
does not say the line ran on data that can tell two translations apart, and the
gap between those two is invisible in every number either instrument reports.
The fix was a different SCENARIO, not a wider window: scenario 27 injects
synthetic azimuth and per-blade rootMOOP, and against it the same stub fails
62 of 63 cases.

The vacuous artifact is committed under `evidence/ColemanTransform/` rather than
deleted. A campaign that keeps only its greens has no record of what its
instruments looked like while they were lying.

## 2026-08-10 — E1.2 was not deferrable, and unit #2 is where it stopped being

`ColemanTransform` is five lines of arithmetic. Its C++ disagreed with the
Fortran by ~1 ULP on 105 of 200 random inputs.

The cause was not the translation. At Release (`-O3`) gfortran contracts
`cos(a)*r1 + cos(b)*r2 + cos(c)*r3` into 4 `vfmadd` instructions — confirmed by
objdump on the shipped `__functions_MOD_colemantransform`, 18 in
`Functions.f90.o`, 103 across `libdiscon.so`. Isolated by compiling the same
subroutine five ways: `-O3` without the flag differs on 105 of 200, `-O3` with
it on 0 of 200, `-O2` on 0 of 200 with or without.

`vit.yaml` had declared `-ffp-contract=off` since setup. `CMakeLists.txt` never
passed it. STATUS.md recorded the gap honestly and deferred it because closing
it means recapturing the baseline — which was true, and which is exactly why it
could not be deferred past the first unit whose evidence depends on it.

Closing it moved 489,734 of 5,252,000 gate values across 90 of 351 channels.
`baseline_arrays/` was regenerated from clean pre-integration source (20 of 27
files changed) and E3.2 re-run against the new set, so the campaign's red
evidence and green evidence describe the same baselines rather than two
different ones.

**`scripts/assert_fp_contract.sh` asserts two things, because either alone
passes while the requirement fails.** The flag is on the build line CMake emits,
AND the linked library holds zero FMA instructions. The first can be satisfied
by editing a file; the second cannot be satisfied by declaring anything. It is
whole-library rather than per-file on purpose: a per-file check goes green the
moment the file under translation is clean and says nothing about the next unit.

The C++ compiler needed it too, and only after integration — `vit integrate`
adds `enable_language(CXX)` and a `.cpp` to SOURCES and sets no CXX flags, so
the translation would have compiled at bare `-O3` and could have reintroduced
the same contraction from the other side, after the Fortran fix had been
verified. "Every compiler" is the criterion.

## 2026-08-10 — the same argument, the same mistake, two independent generators

`rootMOOP` is `REAL(DbKi), INTENT(IN) :: rootMOOP(3)`. Three of this campaign's
generators had to render it:

    interface_gen      kernel bridge + integration wrapper   `double*`         correct
    vit test-validate  the harness's Fortran side            `REAL(C_DOUBLE), VALUE`
    loop vitbridge     the harness's C side                  one double, read three deep

Both wrong ones dropped the rank of an EXPLICIT-SHAPE array — the shape that
carries its extent in its own declaration and therefore needs no synthesised
size parameter. Each generator handled assumed-shape (extent arrives as `n_x`)
and assumed-size (extent declared in vit.yaml) and fell through to the scalar
branch for the third case.

**Neither was found by review.** The first was caught by gfortran, and only
because the bridge `USE`s the module and so is type-checked against the real
interface. The second produced 256 failures out of 257 — loud, but by luck: both
sides read three doubles out of a one-double object at two different stack
addresses, so they disagreed about elements nobody had given them. The identical
defect on an INTENT(OUT) array would have compared one element of three and
reported a clean pass on a translation whose other two were never looked at.

Both fixed at the source with red-tested regression tests (X2), not worked
around here. The loop fix is `translation-loop@8641fbc`; the campaign's pinned
checkout moved `d58a418 -> 3c88913` and `.loop_rev` with it, so `loop_rev` in
every artifact still names the instrument that produced it.

**VIT was NOT upgraded**, and that is the change deliberately not made. This
workspace's VIT is at `d85b33b`; the pinned loop repo expects a newer one (it
cites VIT dev note `202608092223` for a per-unit harness directory this VIT does
not use). Swapping VIT mid-unit would mean this unit's extract/verify/integrate
evidence came from two different tools. `scripts/harness.sh` reconciles the two
directory layouts instead, in a committed script rather than in steps somebody
remembers to repeat.

## 2026-08-10 — what a post-integration harness can actually measure

E4.5 asks for the generated differential harness to be re-run against the
integrated build. Run literally, it does not link: the integrated
`colemantransform.cpp.o` and the harness's own copy of the translation are two
definitions of one symbol.

Removing the collision exposes the real limit. After integration the Fortran
`ColemanTransform` **is** the C++ — the body is a wrapper calling
`colemantransform_c`. There is no independent Fortran reference left in the
build, by construction, so no harness against the integrated build can compare
the arithmetic. One that appears to is comparing the translation with itself.

What remains, and is covered nowhere else, is the INTEGRATION WRAPPER: the
marshalling between the Fortran caller and the C++ callee. That is a generated
bridge, and two other generated bridges in this same campaign were found
dropping this unit's array rank. So the post-integration run routes the
reference side through the real wrapper and the got side straight to the
translation — identical arithmetic on purpose, so that the only thing a failure
can mean is that the wrapper corrupted an argument.

Red-tested: swapping the wrapper's two INTENT(OUT) arguments failed 217 of 217
cases; reverting restored 217/0. The artifact carries a `measures` field saying
this in full, because `checked 217 failed 0` is otherwise indistinguishable from
the pre-integration run that compared against real Fortran, and it is not that.

## 2026-08-10 — three of the gate's 27 scenarios execute no controller code

Not a unit finding, and recorded here because it bears on every number this
campaign reports. Scenarios 10, 14 and 24 run **zero** lines of Controllers.f90,
ControllerBlocks.f90, Filters.f90 and Functions.f90. Scenario 13 runs 12 lines
of Functions.f90 and zero of Controllers.f90. All four enter DISCON and read
parameters — DISCON.F90 42%, ReadSetParameters 39% — and stop.

Scenario 14 prints `Scenario 14: PASSED`, and 11 of its 13 output channels are
one repeated constant. Its 104,000 values are counted in the gate's 5,252,000
and constrain nothing about the controller.

This is E3.3's stated failure mode — "no rejected runs, no all-constant channel
sets" — and E3.3 is still `manual`. It is now measured rather than suspected, so
it belongs to whoever closes E3.3 rather than to this unit. The gate's compared
count is not wrong; it is 5,252,000 values of which some number are inert, and
nothing currently says which.

## 2026-08-10 — the invariant layer asks to be edited, and the hash forbids it

**For the Driver, as a proposed amendment to the method rather than to this
campaign.**

`RUNBOOK.md`'s invariant layer says: "`Status here` starts at `inherited,
unexercised` for every rule. Update it to `exercised at unit #N` the first time
the rule actually bears on the work. Rules still unexercised after N units are
the P4 report."

It also says, four paragraphs above, that editing that layer changes
`invariant_hash` and the Driver reads the change as `method_amendment_proposed`.

Those cannot both be followed. Verified rather than assumed:
`loop/bootstrap.py:143` hashes everything between the front matter and
`\n---\n\n# Runbook — target layer`, which is exactly the block containing the
`Status here:` lines. Writing `exercised at unit #2` on P9 would change the
hash and escalate a routine bookkeeping update as a proposed change to the
spec. The unit's instructions said not to modify the invariant layer, so it was
not modified — verified unchanged at `e4e6b553a5588907` by re-running that same
function.

Two ways out, both for the Driver to choose between: exclude the `Status here:`
lines from the hashed body, or move the status ledger out of the invariant layer
into a target-layer table. The second is cheaper and keeps the hash meaning
exactly one thing.

Meanwhile, so the P4 report is reconstructible without editing anything, the
rules unit #2 actually bore on:

    P3   the kernel's first green named 4,725 compared values and could not
         go red; the stub artifact is committed
    P4   vit.yaml comments restored from the pinned revision, not retyped
    P7   the oracle is the original Fortran -- and after integration it no
         longer exists in the build, which is what limits E4.5
    P8   E1.2 became a disposition (close it) rather than a blocker
    P9   coverage said the line ran 7,998 times; it ran on zeros
    C1   plan.json's bridge_feasible prediction confirmed against the built
         wrapper, not against the declaration it was derived from
    C2   coverage data did not exist; scripts/coverage.py now produces it
    C3   extraction works; the recorded blocker was a dead call site
    C12  the vacuous kernel recorded with its passing artifact before the fix
    X2   two generator bugs fixed at the source, not worked around
    X3   VIT deliberately NOT upgraded mid-unit
    X4   every green red-tested on first use: kernel, gate, post-integration
         harness, and the E1.2 assertion itself
    E1.2 closed
    E3.5 baselines regenerated
    E4.5 the post-integration harness, and what it cannot measure
    E4.6 mutation 35/35

    NOT exercised, and worth naming: P5 (no gap was closed by addition),
    P6 (nothing absent had to be rendered), E1.3, E1.5, E1.6, E2.x, E3.3,
    E3.4, E3.6, E5.x.

## 2026-08-10 — --timeout-s raised to 7200, with the basis measured not guessed

The ColemanTransform session hit the 3600 s default. Its own commit timestamps
split the hour, so the number is derivable rather than chosen:

    17:45:33  dispatched
    18:15:23  Phase 1/3 -- closed E1.2, regenerated the baselines it invalidated
    18:40:50  ColemanTransform translated, verified four ways, integrated
    18:44:07  state commit
    18:45:33  killed by the timeout, ~90 s after its last commit

So roughly **30 minutes of one-time infrastructure** and **~28 minutes of unit
work**. Telemetry agrees in shape: 100 events, 29.6 minutes of bracketed command
time, dominated by extract (28), test-validate (20) and gate (18).

The infrastructure half does not recur, so it says almost nothing about a
typical unit -- which is exactly why 3600 cannot be defended by "it nearly
fit". What it does say is that a unit's own cycle here is ~28 minutes for a
SMALL mirror unit with five scalar/array arguments, and the campaign this method
comes from recorded a 4.8x upward cost trend across its run.

**7200 s.** Roughly 4x the observed unit cycle, which absorbs that trend without
being unbounded. Not a default change: `run_campaign.py` still defaults to 3600,
because one campaign's single measurement should not silently retune every other
campaign. It is passed explicitly at launch and recorded here.

A timeout is no longer a run-ender in any case: `Driver.run_unit` now catches at
the dispatch, records the unit with its cost as UNKNOWN rather than $0.00, and
escalates `session_failed`. The timeout is now a bound, not a cliff.

## 2026-08-10 — the campaign's VIT was 89 commits stale; reconciled at unit 1

`vit-dev` sets `PYTHONPATH=/workspace/vit`, which is `~/Artifacts/vit_translation/vit`
— NOT the canonical checkout the handoff named. So every `vit extract /
translate / verify / integrate` in this campaign ran from a tree 89 commits
behind canonical, and ColemanTransform's evidence already did. Merge-base
`10f7cd4`; 1 campaign-only commit (`d85b33b`) plus 37 uncommitted lines.

Nobody checked which VIT the container imports. The answer was one `git log`
away throughout.

Those 89 commits are the post-replication work whose purpose was removing
fails-green defects from VIT — the redtest `candidates()` fix, the check
registry with its deferral list emptied, reachability's `compile_references`,
exponent grouping, the locale fixes. Running a campaign about silent
verification failure on an instrument known to contain silent verification
failures is self-defeating. Reconciling at unit 1 costs a rebuild and a
re-verify; at unit 40 it would be unaffordable and the instrument would have to
be defended rather than fixed.

Merged to `d07a716`. The one conflict is the argument in miniature: both sides
had independently fixed the SAME explicit-shape-array bug, and canonical's was
strictly better — ours hardcoded `REAL(C_DOUBLE)` for every REAL array, which IS
the kind defect canonical fixed with `_iso_c_for()`, so keeping ours would have
traded a caught rank bug for an uncaught precision one on every `REAL(ReKi)`.
The Makefile link fix was superseded the same way and removed rather than kept
alongside.

Re-verified after the merge rather than assumed: `vit status` and `vit parse`
work (`rootMOOP: REAL(8) INTENT(IN)(3)`, rank preserved), the integrated build
gates 5,252,000 / 0, and `vit_harness` generates 217 cases.

`vit_rev` is now stamped into harness and mutation artifacts beside `loop_rev`,
via a `.vit_rev` pin file because `vit-dev` has no git. Every instrument that
produces evidence should say which version of itself produced it — the loop
stamp caught the loop's own drift on its first live run, and this one would have
caught the 89 before the campaign started.

## 2026-08-10 — ColemanTransform's disposition cleared for re-dispatch

The session recorded `integrated`. The done-condition, evaluated against exactly
what it committed, returns INCOMPLETE at 12 of 13: P12 fails because
`mutation/ColemanTransform.json` carries the pre-fix schema (`total`/
`equivalent`) that P12 cannot read. So the plan asserted a closure the loop had
never verified — the session set the disposition, and the loop, having crashed
on the timeout, never checked it.

`next_unit` selects on `disposition`, so the field had to be cleared for the
unit to be walked again. Evidence is kept: those artifacts exist and are real.
What is NOT kept is the claim that they add up to a closed unit.

Not waived. DECISIONS.md already records that mutation is mandatory for all 69
at a 1.0 threshold; closing unit #1 at 12/13 would make its evidence permanently
weaker than every unit after it, which is the specific thing that commitment
forbids. The artifact must be regenerated by the tool — reset_to_clean ->
generate -> mutate -> restore — not hand-edited to the names P12 wants. A number
I believe is genuine is still fabricated if I type it in rather than measure it.

## 2026-08-10 — unit #2, second pass: the closure measured, and three scripts
   that were wrong the whole time

The unit was re-dispatched to regenerate one artifact. It ended up re-measuring
every layer, because the first thing built was the thing that had been missing:
a way to RUN the done-condition instead of guessing at it.

### `scripts/done_check.py` — run the condition, do not predict it

The previous session set `disposition: integrated` and the loop, crashing on its
timeout, never checked. The claim stood in `plan.json` for four commits. Nothing
in a unit session could have caught it: the done-condition lives in the loop
repo, is evaluated by the Driver, and a session has no way to ask it.

So it is asked now. `done_check.py` imports `loop.done.DoneVerifier` and builds
`DoneConfig` with **exactly** the four globs and four state files
`scripts/run_campaign.py` builds — copied from that call site, not paraphrased
from the docstrings, because a checker that approximates the checker is a second
opinion, not a check. It reads and never writes.

At the start of this pass: INCOMPLETE, 10 of 13. P12 FAIL, P2 dirty (my own
untracked file), P3 NOT_EVALUABLE (the cleared disposition). That is the real
starting position and it was not knowable before.

The 12-of-13 recorded in the previous entry and this 10-of-13 are the same
verdict counted at different moments; the extra two are artifacts of the
re-dispatch itself, not new defects.

### P12: a real 35/35 that the check could not read

`vit_mutate.py` wrote `{total, equivalent}`; `loop/done.py` P12 reads
`{mutants, equivalent_declared}`. `int(d.get("mutants", 0) or 0)` is 0, so P12
reported `mutation_no_mutants: the operator set reached nothing in this unit`
about an artifact recording 35 mutants, all killed.

**Failing with the wrong reason is worse than failing.** It failed safe, which
is the direction that matters least here: a session reading that message goes
looking for missing operators in `cppmutate.py` and finds nothing wrong, because
nothing is wrong there. Fixed at the source in the loop repo (`13265e7`), which
now emits both spellings — the old pair for a human reading the file, the
canonical pair for the loop — rather than dropping names that campaigns already
have on disk. Regenerated here by the tool: 35 mutants, 35 killed, score 1.000,
baseline verified green first.

**33 of the 35 kills are behavioural.** Two `compare_op` mutants are scored
`killed (no compile)`. A translation with no comparison operator in it has
nothing for that operator to reach, and counting a build failure as a kill is
defensible — but it is not the same evidence as a case mismatch, and 35/35 reads
as though it were. Recorded in `plan.json`'s `observability`, since a mutation
score is only as good as what the reader thinks it says.

### Three defects in this campaign's own scripts

All three had one shape: **a stale artifact on disk made a broken run look like
a working one.** `harness/ColemanTransform.json` from the first pass sat there
saying `199 checked, 0 failed`. Three runs today failed to overwrite it, and the
first two were read as successes — including one where the link had died and I
read the previous session's numbers back to myself as if they were new. The tell
was `loop_rev: 3c88913-pinned` in a file a `ebce989` instrument had supposedly
just written. The version stamp caught this, exactly as the entry above hoped it
would; without it there was nothing to notice.

The general rule, which belongs to the method and not to this campaign: **an
artifact that a run failed to write is indistinguishable from one it wrote,
unless something in it identifies the run.** Timestamps do not survive git.
Every evidence-producing instrument should stamp its own version, and a reader
should check the stamp before believing the numbers. `_harness_stamp.py` now
stamps `vit_rev` beside `loop_rev` — the post-integration artifact measures the
integration WRAPPER, the one thing in this pipeline VIT generates, and it was
the only artifact not recording which VIT that was.

1. **The skew `harness.sh` existed to reconcile had closed.** VIT `d07a716`
   brings in dev note `202608092223`'s per-unit test directory, so
   `vit test-validate` now writes the Makefile where `vit_harness.py` already
   looked. The script exited 1 with "vit test-validate did not write Makefile"
   while the Makefile sat one level down, timestamped four seconds earlier by
   that same command. Both layouts are now handled and which one ran is
   PRINTED. The old branch is kept rather than deleted: the artifacts already
   committed for this unit came from the old layout, and a script that cannot
   reproduce them stops being able to say where they came from.

2. **The post-integration LIBS extraction passed the words `LIBS` and `+=` to
   the linker.** It grepped `^LIBS =` through to the first blank line and
   stripped the prefix per line. The new VIT's Makefile ends the block with a
   second assignment, `LIBS += -lgfortran -lm`, which `^LIBS =` does not match,
   so two makefile tokens arrived at `ld` as filenames. Replaced with asking
   `make` to evaluate its own variable through a two-line generated include.
   A hand-rolled parser that knows one assignment operator and not the other
   will break again on the next generator change; `make` already knows.

3. **A red run could not be stamped.** `./test` exits non-zero when cases
   mismatch, and under `set -e` the script aborted one line before
   `_harness_stamp.py`. The red-test artifact was therefore written with
   `against: "translation"`, no `measures`, and no revisions — the run whose
   artifact most needs to say what it measured was the only one that could not.
   The stamp now runs whenever a parseable artifact exists, a link failure is
   still a hard stop, and `--red-test "<perturbation>"` records the
   perturbation and INVERTS the verdict, so a red test that stays green is
   itself a failure. The previous pass's red-test artifact carries a `red_test`
   block that was added by hand; this one was written by the tool.

### The pre-integration harness could not link, and that is `reset_to_clean`'s
    boundary showing

`reset_to_clean.sh` restores the Fortran body and deliberately leaves
`CMakeLists.txt` alone. After a unit has been integrated once, the build target
still compiles `rosco/controller/src/<stem>.cpp`, so the tree it produces is a
hybrid: pre-integration Fortran, post-integration build. The differential
harness compiles its own copy of the translation and links the build's objects,
so `<stem>.cpp.o` is a second definition of the same function and the link dies.

This could not have happened on the first pass — nothing had been integrated
yet, so the object did not exist. **It will happen on every unit from here.**

`harness.sh` now drops this unit's own object from the generated LIBS, and only
its own: other units' `.cpp.o` files are reached through their `_c` bridges and
are part of the reference build. The edit goes into the Makefile rather than
onto one `make` command line, because `vit_mutate.py` runs `make` itself — a fix
that lived only in the shell script would leave every mutant failing to link,
scored `killed (no compile)`, i.e. a 1.000 that measured nothing. `vit_mutate.py`
refuses to score when the baseline will not build, so it would have failed
loudly rather than lied. The point is that it must not have to.

The better fix is to make `reset_to_clean.sh` revert `CMakeLists.txt` too, so
"clean" means the pre-integration BUILD and not merely the pre-integration
SOURCE. Not made here. That script is red-tested against ten assertions and the
second pass of unit #2 is not where to re-open it; recorded in STATUS.md as an
open decision for before unit #3.

### What the re-measurement found: nothing wrong with the translation

Every layer was re-run under VIT `d07a716` / loop `ebce989`, and every one
agrees with the first pass. `vit integrate` regenerated `Functions.f90`,
`CMakeLists.txt` and `colemantransform.cpp` byte-identical to what `d85b33b`
produced — `git diff` over the three is empty. Kernel 63/63 on live inputs, the
zero-writing stub still fails 124 field entries, harness 199/0 against clean
Fortran, mutation 35/35, post-integration 199/0 with the wrapper swap turning it
199/199 red, gate 5,252,000/0 with the perturbation moving 124,353.

Worth stating plainly, because the re-run was expensive and found no arithmetic
defect: **that is the result, not a wasted pass.** The 89-commit VIT reconcile
was justified on the grounds that a campaign about silent verification failure
must not run on an instrument known to contain silent verification failures. The
evidence that the reconcile did not disturb this unit is the thing that lets the
first pass's conclusions stand — and it turned up three defects in the
campaign's own scripts that the next unit would have hit anyway.

### The bridge prediction

CONFIRMED, and now confirmed twice by two generators. All five arguments cross
with no decomposition; details in `plan.json`'s `bridge_feasible.observed`.

## 2026-08-10 — for the Driver: the invariant layer's `Status here` cannot be
   maintained by a unit session

The invariant layer of `RUNBOOK.md` says of every rule:

> `Status here` starts at `inherited, unexercised` for every rule. Update it to
> `exercised at unit #N` the first time the rule actually bears on the work.
> Rules still unexercised after N units are the P4 report: inherited, never
> tested against this target.

That instruction lives INSIDE the invariant layer and can only be carried out by
editing the invariant layer — which changes `invariant_hash`, which the Driver
reads as a proposed amendment to the method and escalates. A unit session is
told not to edit that layer. So the field is unmaintainable by the only actor
positioned to know the answer, and after two closed units all 44 rules still
read `inherited, unexercised` while at least a dozen demonstrably bore on the
work. This pass alone exercised C1-C11, P3, P7, P9, X2 and X4.

The P4 report the field exists to produce is therefore empty-by-construction
rather than empty-by-measurement, which is the failure this method is about.

Not fixed here, because fixing it IS the escalation. Raised as a proposed
amendment. Three shapes worth considering, in preference order:

1. Move `Status here` out of `RUNBOOK.md` into a generated sidecar
   (`runbook_status.json`) that a unit session may write and the invariant text
   references. The rule list stays hashed; the observation about it does not
   need to be.
2. Exclude `Status here:` lines from the `invariant_hash` computation, so
   updating them is not an amendment.
3. Have the Driver derive it, from the predicates each unit's `done_check`
   evaluated and from what the session's commits touched — no session writes it
   at all. Weakest of the three: it can see which contracts were checked, not
   which prohibitions were felt.

Until one of them exists, the field should be read as "nobody could write this",
not as "these rules were never exercised".

## 2026-08-10 — ceiling $3000, per-unit cap $25, both settled before unit 2

Both parameters are recorded in `run.json` and the Driver REFUSES to resume a
run whose limits changed, because carrying spend across a changed ceiling
misreports both the spend and the headroom. So they had to be decided together
and decided now: settling the total today and the per-unit cap at unit 10 costs
the same restart twice. One unit is closed, so the restart is free; at unit 40
it would not be.

**Total $3000, not $1000 and not $2000.** The total only functions as a guard
BELOW `68 x per_unit_cap`. At the old $35 cap that product is $2,380, so $2000
sits inside the nominal range and could bind from ordinary spending — the exact
failure we are paying a restart to avoid. Above it, the total stops being a
truncation risk and becomes what a total is for: catching retry storms. A retry
is a second dispatch, so the worst case is 68 x $50 = $3,400 at the new cap, and
$3000 halts that.

**Per-unit cap $25, down from $35.** A generous cap SILENCES the per-unit
signal: `unit_overran` almost never fires, so costs get watched by hand instead
of reported. ColemanTransform closed at $11.885, so $25 is ~2x the observation —
generous enough not to false-fire, tight enough to catch an outlier. Exceeding
it escalates AFTER the fact rather than killing work, so a tight cap buys signal
at no cost. 68 x $25 = $1,700 nominal, comfortably inside $3000.

**$11.885 is a FLOOR, not an estimate**, and the projections above are built on
it knowingly. That dispatch found its translation and integration already
committed; the cold run that did that work timed out and its spend was never
recorded, so nothing in the ledger reflects a full cold unit. The accounting
restarts here: `run.json.pre-ceiling-change` preserves the old record and
`units.jsonl` keeps ColemanTransform's row, so the campaign's true spend is
$11.885 plus whatever this run reports.

Unit 2's cold cost will be reported when it lands. Not as a gate — it is
ColemanTransform's sibling and among the simplest units in the plan, so it is
another floor rather than a representative sample — but because the gap between
warm and cold is the number that makes every later projection honest.

## 2026-08-10 — unit #1: the verdict was right, the reason was about a
   different generator, and the matrix could not have known

`plan.json` predicted `AddToList` cannot cross the C bridge, basis
`list: c_alloc_inout does not cross (INTENT(INOUT) ALLOCATABLE needs an
ALLOCA...)`. **The verdict is confirmed. The basis is refuted.** Both halves
were measured; neither was argued.

### Confirmed: the generated bridge compiles and does nothing

`vit interface` emits `INTEGER(C_INT), INTENT(INOUT) :: list(*)` and
`INTEGER(C_INT), VALUE :: n_list`, under a wrapper whose dummy reads
`INTEGER(4), INTENT(INOUT) :: list(:)` — the ALLOCATABLE attribute gone.
`fortran_parser` had recorded `is_allocatable=True`; `generate_fortran_wrapper`
attaches `, ALLOCATABLE` only for `is_alloc_return` dummies and dropped it
without a word.

`AddToList`'s whole contract is `allocate(clist(isize+1))` → copy →
`move_alloc(clist, list)`: it replaces the caller's data pointer AND its extent.
A bare pointer and a by-value length carry neither. Three implementations were
built against one driver on the live ROSCO path (both call sites `ALLOCATE`
first, then append):

    orig  A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17
    vit   A1 size=4         A2 size=4         A3 size=4
    cfi   A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17

The array never grows. The appended values are written one past the end of the
caller's heap block. gfortran said nothing, g++ said nothing, the run did not
crash. **A wrapper that compiles and does nothing is worse than one that does
not compile** — the second is caught by the build.

### Refuted: the limit is VIT's, not Fortran's

Fortran 2018 permits an `ALLOCATABLE` dummy in a `BIND(C)` interface. The C side
receives the caller's own `CFI_cdesc_t *`, and `CFI_deallocate`/`CFI_allocate`
through it do exactly what `move_alloc` does. A hand-written CFI bridge
reproduced the oracle **byte for byte on both branches**, including
`move_alloc`'s reset of the lower bound, under gfortran 13.3 / g++ 13.3.
Red-tested: one perturbed token in the CFI body turns the probe red.

So `c_alloc_inout does not cross` was a true sentence about the tool wearing the
grammar of a claim about the language. The distinction is not academic — it is
the difference between "close this unit as impossible" and "VIT is four
`plan.json` entries short of a feature it can have."

### Why the matrix could not have known: it measures a different generator

`tests/test_conformance.py:probe()` fed `bridge` and `compiles` from
`test_validate.generate_fortran_bridge` — the **differential harness's** Fortran
side, linked into a test binary and never into the library. On `AddToList` it
fails with `Actual argument for 'list' must be ALLOCATABLE`, which is where
`[c_alloc_inout] compiles = "no"` came from and therefore where the plan's basis
came from. The matrix never called `generate_fortran_interface_block` or
`generate_fortran_wrapper`, the two generators whose output `vit integrate`
writes into the shipped source, and those compile clean.

**The prediction was right by luck of a different generator failing.** Any cell
whose harness bridge happened to compile would have read `yes` and the campaign
would have integrated a no-op. That is the campaign's own failure shape, sitting
in the file built to prevent it: a column that means less than its name.

### Fixed at the source, both halves, red-tested (X2, P5)

1. `interface_gen.assert_integration_bridgeable` raises `UnbridgeableSignature`
   for an ALLOCATABLE INTENT(INOUT) dummy. Called from the interface-block
   generator, the wrapper generator, and the C++ scaffold — `vit translate` is
   the earliest point at which a wasted body is avoidable. `vit translate`,
   `vit interface` and `vit integrate` on `AddToList` now exit 1 and write
   nothing. **Only INTENT(INOUT) is refused**, because only INTENT(INOUT) was
   measured; INTENT(IN) ALLOCATABLE has the same dropped attribute and no
   campaign has hit one, and inventing a verdict for an unmeasured cell is the
   habit this refusal exists to break.
2. `test_conformance.py` gained an `integrates` column that runs the integration
   generators and compiles their output. Regenerating the matrix moved
   `c_alloc_inout` to `integrates = "refused"` and turned up **`c_complex_in`**,
   a cell every previous column called supported, whose integration wrapper
   emits `COMPLEX(z, C_DOUBLE_COMPLEX)` for an already-COMPLEX dummy and does
   not compile. Five of 39 cells now disagree between the two columns.

Both red-tested by disabling them and confirming the suite fails
(`test_cell_matches_the_record[c_alloc_inout]`, three refusal tests), then
restoring. VIT suite: 930 passed.

**Two probe defects were found first, and both were the same mistake in
miniature.** The `integrates` column initially reported `no` for seven cells:
three because the synthetic module did not `USE vit_conformance_types`, three
because the generated view-populator modules did not exist, one because
`_compiles` matched only `Error:` and not `Fatal Error:` and so reported
`compilation terminated.` — a diagnostic naming no cause. Every one of those was
measuring the probe. They are fixed and the reasons are in the code.

### The gate cannot see this unit, measured twice

`coverage/line_coverage.json` says the body and all five call sites have zero
hits in all 27 scenarios. Confirmed independently of gcov by perturbing BOTH
branches (`= element` → `= element + 1000`, 2 replacements), rebuilding, and
re-gating: **0 of 5,252,000 values moved.**

So `gate/AddToList.json` — 5,252,000 compared, 0 mismatched — is a green that
constrains nothing about `AddToList`. It is committed next to the red test that
says why, because the green alone would be a lie by omission.

### A reading hazard the dead red test exposed

On a `--perturb-*` run, `gate.py` left `verdict` at the COMPARISON's verdict. So
a red test that correctly went red wrote `verdict: FAIL`, and one that failed to
go red wrote `verdict: PASS`. `AddToList` produced the dangerous half: a gate
blind to the unit, filed under `PASS`.

`verdict` on a perturbing run is now `RED_TEST_PASS` / `RED_TEST_FAIL`, and the
comparison's own verdict is kept under `comparison_verdict`. **The new spelling
is deliberately not `PASS`/`FAIL`**: `gate/ColemanTransform.redtest.json` was
written under the old convention, and a reader can now tell which convention a
file uses from the value itself rather than from its date. Verified in both
directions: the `LPFilter` perturbation over scenarios 1/3/6 moved 265,890 of
780,000 and wrote `RED_TEST_PASS` (rc 0); `AddToList` wrote `RED_TEST_FAIL`
(rc 1).

### Disposition: `blocked`, and it escalates

SPEC §8.4 requires escalation when a disposition is `blocked` and the blockage
is in the substrate. It is: VIT does not generate CFI descriptors. Two questions
for the Driver, neither decidable inside a unit session:

1. **Should VIT learn the CFI bridge?** It is demonstrated working here, and it
   is what `AddToList`, `Read_OL_Input`, `ParseInAry_Opt` and `ParseDbAry_Opt`
   are waiting on — 4 of the 5 `bridge_feasible: no` entries in this plan. It
   also has to reach `test_validate`'s bridge and the loop's `vitbridge` before
   a blocked unit can produce a mutation score, so it is three generators, not
   one.
2. **Can a unit no scenario reaches ever close?** P9 can only be satisfied
   vacuously here, P12's mutation score cannot be produced, and P12's red-test
   fallback is impossible by construction. `integrated_unexercised` exists in
   the vocabulary for exactly this case and the done-condition has no branch for
   it. Until it does, every dead unit in this plan will end `blocked` for a
   reason that has nothing to do with its translation.

### What was NOT done, and why

No translation was written and none was integrated. The correct CFI translation
exists in `evidence/AddToList/bridge_probe/addtolist_cfi.cpp` and was **not**
promoted into `translations/`: integrating it would need the interface block and
wrapper hand-written into `ROSCO_Helpers.f90`, which is the generator's job
(X1/X2), and `reset_to_clean.sh` and `restore_integrated.sh` select integrated
files by looking for a `<name>_c(` bridge that would not exist. Shipping it by
hand would buy one function and leave the next three at the same wall with the
tooling still unable to see it.

### The done-condition, run rather than predicted

`python3.12 scripts/done_check.py AddToList --baseline 2ef6d0d` →
**INCOMPLETE, 11 of 13.**

    P1..P8   ok
    P9       ok   -- on the vacuous gate green, which is the point
    P12    FAIL   mutation_missing: no mutation/AddToList.json
    P11    FAIL   harness_not_rerun: no harness/AddToList.postintegration.json
    P13,P10  ok

The two failures are the two artifacts that cannot exist: both the differential
harness and the mutation score are generated from the bridge that does not
cross. There is nothing to fix that is inside this unit — which is escalation 2,
now measured rather than argued.

`--baseline` is required: `done_check.py` infers the window from the first
commit touching `plan.json`'s `translation`, and a blocked unit has none.

Recorded here rather than left to be re-derived, because the alternative is a
later session reading `blocked` beside a green STATUS and assuming the condition
was never run — which is exactly what happened to unit #2.

## 2026-08-10 — unit #1, second dispatch: the prediction is refuted, because
   the tool learned the thing it was refusing

The unit was re-dispatched with exactly two unmet conditions: `P12
mutation_missing` and `P11 harness_not_rerun`. Both are generated from the
bridge, so "address exactly these" had one honest reading — build the bridge.
That is escalation 1, answered by doing it rather than by deciding it.

### What crossed, and how

```fortran
    INTERFACE
        SUBROUTINE addtolist_c(list, element) BIND(C, NAME='addtolist_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), DIMENSION(:), ALLOCATABLE, INTENT(INOUT) :: list
            INTEGER(C_INT), VALUE :: element
        END SUBROUTINE addtolist_c
    END INTERFACE
```

The dummy is declared exactly as the original declares it, **no extent
parameter is emitted** — the descriptor carries the extent, and a separate
`SIZE()` is stale the moment the callee reallocates — and the C side receives
`CFI_cdesc_t*`, which IS the caller's descriptor.
`CFI_deallocate`/`CFI_allocate` through it do what `move_alloc` does.

Three generators, as the escalation predicted it would take: `interface_gen`
(what ships), `test_validate.generate_fortran_bridge` (the differential
harness's Fortran side), and the loop's `vitbridge`/`emit` (the C++ side).
Missing any one of them leaves a unit that integrates and still cannot produce
a mutation score — which is precisely how this unit was blocked.

**Nothing about the integration is hand-written.** The first dispatch declined
to promote the working CFI body into `translations/` because the interface
block and wrapper would have had to be hand-written into `ROSCO_Helpers.f90`
(X1/X2). They are generated now, which is what made promoting it legitimate.

### Refused, still, and why the list is the point

`assert_integration_bridgeable` now refuses a SCALAR allocatable, rank >= 2, a
CHARACTER or derived-type element, and any type/kind the interface cannot
declare as the actual's own (LOGICAL, COMPLEX). **Every one of those is refused
for being unmeasured, not for being impossible**, and each exception message
says so. Rank >= 2 is the sharp case: the descriptor carries any rank, and the
C++ body's indexing convention is exactly where a silent error would live, so
the refusal is about what nobody has run against an oracle.

### The mutation score went 0.739 -> 0.920 -> 1.000, and only the middle step
    was about the harness

First run: 17 of 23, six survivors. Two of them were real and neither was a
weakness of the input set:

* `malloc(isize + 1)` -> `isize - 1`: heap under-allocation, then a write past
  the end. No value comparison can see it.
* `memcpy(..., isize + 1)` -> `isize + 2`: a copy past the buffer, same.

Both existed because ONE quantity — `SIZE(clist)` — was written three times: the
allocation, the new upper bound, and the copy length. Two of the three
restatements were therefore unobservable by construction. Naming it once
(`nsize`) leaves a single site, and that site decides the extent the caller
sees; both mutants are then killed by the extent comparison.

A third came from writing Fortran's `do i=1,isize` as a 0-based C loop:
`i < isize` perturbed to `i <= isize` writes a slot the next statement
overwrites, and is invisible. Written 1-based, as the Fortran has it, the
perturbation goes the other way and leaves an element uncopied — which the
comparison sees. **The more literal transcription is the more observable one.**

The two survivors that remain are DECLARED equivalent, with the reason in
`mutation/AddToList.equivalences.json`: `CFI_allocate`'s `elem_len` argument is
ignored unless the type is `CFI_type_char`, `CFI_type_ucs4_char` or
`CFI_type_struct` (F2018 18.5.5.5). No input can distinguish them — which is
what "equivalent" has to mean, and is not what was true of the two above. The
file records the removed pair too, so the distinction survives the commit.

### The unallocated actual is an input the generator had no rule for

`.NOT. ALLOCATED(list)` is not extent 0, and it is the whole `else` branch.
`_extent_plan` ranges over 1, 3, 4, ... and would never have produced it, so
the branch would have been unreachable by every case — 25 of the 43 cases here.
Modelled as a two-valued flag it becomes R2's business, and R2 already
guarantees every declared value appears. No new rule, an existing rule given
something to act on.

Measured: perturbing the allocated branch turns 18 of 43 red, the unallocated
branch 25 of 43, and 18 + 25 = 43. **No case is inert.**

### Two defects found in generators on the way, both fixed at the source (X2)

1. `vit_translated.h` is included FIRST by every generated `.cpp`, ahead of the
   translation's own includes, so a declaration naming `CFI_cdesc_t` made the
   header uncompilable. The interop include is now added to it on demand.
2. **Every file `vit integrate` generated carried `// After verification:
   <name> kernel PASSED`.** Integration does not run a kernel, does not read a
   kernel result, and cannot know whether one exists. On this unit the sentence
   was flatly false: there is no kernel and there cannot be one, because no
   scenario reaches the function. A generator asserting a verdict it never
   checked, shipping inside the translated source, is the exact failure this
   campaign exists to find — and it was found by reading the file it had just
   written, not by any check.

   `rosco/controller/src/colemantransform.cpp` was NOT regenerated, so unit
   #2's "regenerates byte-identical" property no longer holds for those comment
   lines. Recorded in STATUS rather than silently repaired.

### A hand-copied ABI in a shell script

`scripts/harness.sh --post-integration` wrote its shim from a heredoc
containing `void ${UNIT}(double*, double, int, double*, double*)` — ColemanTransform's
parameter list, as a literal, for every unit. This unit would have failed to
compile, which is the good outcome; a unit whose arity happened to match would
have compiled and forwarded the wrong arguments. `scripts/_integration_shim.py`
asks `build_c_params`, so there is one spelling of the ABI rather than two.

### What this unit is verified BY, and what it is not

| layer | result | red test |
|---|---|---|
| differential harness, 43 cases vs clean Fortran | 0 failed | 18/43 and 25/43 red, one per branch |
| mutation | 23/23 behavioural killed, 1.000 | baseline must be green or it refuses to score |
| post-integration harness (wrapper marshalling) | 43 cases, 0 failed | wrapper corrupts an argument -> 43/43 red; green restored |
| gate, 27 scenarios | 5,252,000 values, 0 mismatched | perturbing the integrated C++ moved **0** — `RED_TEST_FAIL` |

The last row has not changed and is the point. **This unit closed without the
gate ever being able to see it.** Escalation 2 is therefore narrower than it
was: a dead unit whose signature crosses now has a path — harness plus mutation
— and a dead unit whose signature does not cross still has none, with
`integrated_unexercised` still unimplemented in `loop/done.py`.

### Instrument stamps: the artifacts were regenerated, not relabelled

The first pass's artifacts were produced by code that was still uncommitted, so
they stamped `22086e8-pinned` / `46a7f4f-pinned` — revisions that did not
produce them. VIT and the loop were committed (`37f8bdf`, `cf885e3`), the pin
files updated, and **every artifact re-run**: the pre-integration harness and
the mutation score from a `reset_to_clean` tree (they measure the clean Fortran
and must not be taken against the integrated build), the post-integration pair
after `restore_integrated`, and the gate last. The RUNBOOK's warning about
stale artifacts reading like fresh passes is the reason this was not skipped.

## 2026-08-10 — for the Driver: one candidate method amendment, and two that
   are not

The invariant layer was not edited, so `invariant_hash` is unchanged. Flagged
here instead, as the dispatch asks.

**CANDIDATE (method).** *A generator may not state a verification verdict it did
not obtain.* `vit integrate` wrote `// After verification: <name> kernel PASSED`
into every file it produced, having run no kernel and read no result. It is the
same defect P3 and P6 describe — a claim with nothing behind it, and absence
rendered as a value — but neither principle reaches a GENERATOR's output, only
a checker's. Nothing in the toolchain checks a comment, and this one shipped in
source for as long as the generator existed. If the method has a rule that
artifacts must be able to name what they measured, its scope should include
what tools emit, not only what verifiers report.

**NOT method, target.** The descriptor bridge itself, and the ordering
constraint that a pre-integration harness and a mutation score must be taken
against a clean tree: both are in the RUNBOOK's target layer, where they
belong.

**NOT method, but worth a second campaign's data.** "Raise a mutation score by
removing restatements of one quantity, and DECLARE only what no input can
distinguish." It reads like a rule and it has been measured exactly once. Two
survivors here were unobservable memory errors caused by writing `SIZE(clist)`
three times; two others are equivalent by the Fortran standard. The distinction
held up on this unit. One unit is not a rule.

## 2026-08-11 — Unit #3, ColemanTransformInverse: three instruments read for what
   they actually say

**The RUNBOOK's liveness recipe cannot be run on a unit whose outputs are
arrays, and nothing said so.** The recipe reads `reference` out of
`kernel/<Unit>/verify_fields.csv` and counts non-zeros; VIT logs a scalar field
with its computed and reference values, and an ARRAY field as a single row with
BOTH value columns EMPTY and `diff=size=3`. For ColemanTransformInverse — one
output, `PitComIPC(3)` — the recipe returns nothing at all. It does not fail; it
produces an empty answer that a session in a hurry would read as "no zeros
found".

The stub run is therefore not a second opinion here, it is the only opinion.
Against a translation reading no argument and writing `0.0`, 61 of 63 cases go
`OUT_TOL`. The 2 that do not are invocations 1 and 2 — simulation start, where
the axis inputs are still zero. That is unit #2's `1-20` window failure mode
surviving as two cases out of 63 instead of all 21, and it is visible only
because the stub was run.

The RUNBOOK's target layer now says this; the recipe is kept beside it, because
it is still the right check for a scalar-output unit.

**A kernel PASS is not a bit-identity claim, and this was read out of the
generated code rather than assumed.** `kv_ipc_real__dbki_dim1`, the comparison
KGen generated for this unit's array output:

    IF (ALL(var == kgenref_var)) -> IDENTICAL
    ELSE rmsdiff = SQRT(SUM((var-ref)**2)/n)
         IF (rmsdiff > kgen_tolerance) -> OUT_TOL  ELSE -> IN_TOL

`kgen_tolerance` is `1.D-14` and `rmsdiff` is ABSOLUTE, not relative. This
unit's outputs are of order 1e-3 rad, so a translation wrong at ~1e-11 relative
would score `IN_TOL`, `numOutTol` would stay 0, and the run would print
`✓ VERIFICATION PASSED: 63/63 passed`. Nothing in that line distinguishes it
from bit-identity.

What makes this unit's kernel evidence bit-exact is not the verdict. It is that
all 63 fields landed in the `ALL(var == kgenref_var)` branch — `IDENTICAL` in
the field log's status column. **Read the status column, not the verdict line.**
This is P3 applied to somebody else's instrument: a green must be able to name
what it compared, and `63/63 passed` names a tolerance, not an equality.

**One argument of this unit is invisible to both bit-exact instruments.**
`aziOffset` is 0 at all five call sites in all 27 scenarios: `IPC_aziOffset` is
`0.000 0.000` in all 14 `Examples/DISCON*.IN` and no scenario patches it,
`AWC_phaseoffset` is `0.000000000000` in all of them and the single scenario
that patches it (15) patches it to `'0.0'`, and `Controllers.f90:699` passes the
literal `0.0_DbKi`. A translation that ignored `aziOffset` entirely would pass
the kernel 63/63 and the gate 5,252,000 of 5,252,000.

The differential harness is what covers it — `R6_reference_literals` varies it,
and the three `aziAngle + aziOffset` → `-` mutants die in 81 of 257 cases each.
Those 81 cases are the entirety of what constrains that argument in this
campaign. This is P9 at argument granularity rather than line granularity: the
line is executed 100,000+ times per scenario and one of its inputs is a
constant throughout. Coverage cannot express that, and the gate's red test
cannot either — perturbing the output moved 389,644 values, which says the unit
is seen, not that every argument is.

**Not escalated, because the harness already answers it.** Recorded so that the
next unit with a parameter constant across all scenarios is not surprised, and
so the campaign's ledger can say which arguments its bit-exact layers bound.

**`vit check -f <file>` scopes its cross-source checks to the FILE, not the
function.** It reported `minval-endpoints` and `array-section-row` against this
translation; both come from `interp1d`/`interp2d` elsewhere in `Functions.f90`,
lines 132-271, and neither intrinsic appears anywhere in
`ColemanTransformInverse`. `--function` only sets the report header — its own
help text says so. On a 900-line multi-procedure source every finding has to be
re-attributed by hand before it means anything, and a session that trusts the
count will either chase two ghosts or start ignoring the checker. Not fixed
here: it is a VIT change, and X2 says fix it rather than work around it, but it
belongs to a VIT session and not to this unit's cycle. Left as an open item in
STATUS.md.

**Extraction dirties one more file than the RUNBOOK listed.** The scenario's
`Examples/DISCON*.IN` comes back with its `File written using ROSCO version ...
on MM/DD/YY` header line rewritten to today's date, because extraction runs
`vit_sim.py`, whose `write_discon()` regenerates it. Content-free, and it still
blocks `done.py`'s clean-tree predicate. Restored by hand; noted in the RUNBOOK
beside the other extraction leftovers rather than added to
`reset_to_clean.sh`, because that script is about the SOURCE tree and this file
is the gate's input — `gate.py` already owns restoring these.

### What this unit is verified BY

| layer | result | red test |
|---|---|---|
| kernel replay, 63 cases, scenario 27 | 63/63, all `IDENTICAL` (the equality branch, not the tolerance branch) | zero-writing stub → 61/63 `OUT_TOL`; green restored on revert |
| differential harness, 257 cases vs clean Fortran | 0 failed | the mutation score, below |
| mutation | 24/24 behavioural killed, 1.000, 0 declared equivalent (2 nocompile EXCLUDED) | refuses to score unless the baseline is green |
| post-integration harness (wrapper marshalling) | 257 cases, 0 failed | `axTIn`/`axYIn` swapped in the wrapper's CALL → 256/257 red; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `PitComIPC[0]` scaled by 1.000001 → 389,644 values moved; revert → 0 |

Two of the 24 mutation kills are CRASHES, not case mismatches: `[2] -> [2 + 1]`
and `'2' -> '3'` both write one past the end of the caller's 3-element array.
Honest kills — the harness died — but a value comparison alone would not have
seen them, so the behavioural-by-comparison count is 22 of 24. The same
distinction unit #1 drew about `killed (no compile)`, one layer in.

`vit_mutate.py` now EXCLUDES nocompile mutants from the score rather than
counting them (loop `46a7f4f`), so unit #2's `35/35` and this unit's `24/24` are
not the same measurement. Unit #2's included 2 nocompile kills; this one's does
not.

## 2026-08-11 — Unit #4, Conv2UC: a hot line the gate cannot see, and the
   instrument that had to be built before the unit could close

**The differential harness had no string kind, and 20 of 69 units need one.**
`vit_harness.py` printed `UNOBSERVABLE [character-arg] Str: crosses as char* +
length and IS comparable; the generator has no string kind, so this argument is
held constant` — a diagnostic somebody had already written, whose own text calls
the gap "a gap with a fix, not a property of the bridge" — and then died:
`EmitError: Conv2UC: C parameter 'Str' is not in the mapped signature`. So the
report and the behaviour disagreed. It said *held*; it meant *unrunnable*.

The scale is what made this a feature rather than a note. Parsing the plan's 69
units against their Fortran declarations, **20 take a CHARACTER dummy argument**
— `GetPath`, `GetRoot`, `GetWords`, `FindLine`, the five `ParseInput_*_Opt`, and
`DISCON` itself. P11 (differential harness) and P12 (mutation score at 1.000)
are mandatory for every unit and `done.py` has no branch that waives them, so
none of those 20 had any path to close.

Fixed rather than worked around, which is X2 and is also what unit #1 did with
VIT's descriptor bridge. Loop `9bee569`: a `char[]` kind in which a CHARACTER
dummy's length is an EXTENT — the thing the rules already know how to vary — and
its bytes are an array of one-byte elements, which the case stream already knows
how to carry. 8 tests, none of which need VIT; 414 collected; the pre-existing
failures in this environment are unchanged, verified by diffing the failure set
against a `git stash`ed baseline rather than by counting.

Four things in it are worth carrying:

1. **`len_<x>` had to be RECOGNISED as an extent.** `_EXTENT_1D` matches `n_<x>`
   only, so the length fell through to the scalar branch and was VARIED OVER
   ±1e3 — two implementations reading that many bytes out of a buffer sized by
   a different number. It is now matched, guarded on the C type of `<x>` rather
   than on the name.
2. **The string and its length are ONE call argument.** `_order` has to put the
   length in the stream before the bytes, because the buffer cannot be declared
   until its size is read; that is only expressible if one object owns both.
   Adjacency in the C parameter list is asserted, not assumed — a generator that
   separated them would transpose the call rather than fail to compile.
3. **R6, one type over.** The reference's own CHARACTER literals and their two
   collating neighbours are the discrete analogue of a literal and its ULP
   neighbours, and they are what makes `>= 'a'` distinguishable from `> 'a'`.
   Scoped to the UNIT's body, unlike `literals_from`, which reads the whole
   file: a stray numeric literal is mostly discarded by the range filter, but
   every character is inside every string's domain, so `ROSCO_Helpers.f90`'s
   hundred stray literals would have buried the two `Conv2UC` branches on.
4. **Three SHAPES, not just three lengths** — mixed, word-then-blanks,
   all-blank. `LEN_TRIM` and `LEN` are the same number on a string with no
   trailing blank in it.

REFUSED, for being unmeasured rather than impossible, each with its own reason
in the report: a CHARACTER **array** dummy (element width and element count are
two extents where `char[]` has one), and a `char*` with no `len_` beside it.
`FindLine` and the `Parse*Ary_Opt` family all take a CHARACTER array, so that is
the next thing this will need — recorded now rather than discovered then.

**THE GATE IS BLIND TO A PROCEDURE CALLED 1.3 MILLION TIMES, AND THE REASON IS
NOT DEADNESS.** `Conv2UC` converts 4,558,823 characters across the 27 scenarios.
Two perturbations of the integrated C++ were built and gated and each moved
**0 of 5,252,000 values**: `ch - 32` → `ch - 31`, and the guard forced false so
that no conversion happens at all. Both artifacts carry `replacements: 1`,
`perturbed: true`, `reverted: true`, `revert_verified: true`.

The cause is that every value this unit produces is consumed by an equality test
against another of its own outputs. `FindLine` upper-cases the expected
parameter name at `ROSCO_Helpers.f90:1106` and the file's word at `:1118` with
the same function and compares them; a perturbation lands on both operands, and
equality survives. The no-conversion run shows it survives non-injective damage
too, on the inputs these scenarios carry.

This is the third distinct shape of P9 in four units, and it is the one that
most directly attacks the coverage argument. #1 was a line no scenario reached.
#3 was an argument that is a constant in every scenario. **This is an executed
line — one of the hottest in the tree — whose result is cancelled downstream.**
A hit count says a line ran. It says nothing about whether anything downstream
can tell two implementations apart, and `coverage/line_coverage.json` cannot
express the difference. The `--perturb-to 'if (false && ...)'` form is the
general instrument for asking, because it makes the unit a no-op rather than
testing one guess about what a defect would look like; it is in the RUNBOOK.

**Removing a restatement raised the mutation score from 0.696 to 1.000, and it
is the second measurement of that rule rather than the first.** The Fortran's
loop bound is `LEN_TRIM(Str)`. Transcribed literally that is
`while (len_trim > 0 && Str[len_trim - 1] == ' ') --len_trim;` — six mutable
sites computing a quantity nothing downstream can read, because every character
past `LEN_TRIM` is a blank and a blank is never converted. All six survived, and
FIVE survived as out-of-bounds reads: `Str[len_trim + 1]`, `Str[1 - len_trim]`,
and `len_trim >= 0` reading `Str[-1]`.

Declaring those equivalent would have recorded a blindness in the harness as a
property of the mutants — the exact distinction unit #1 drew. Removing the
restatement left one loop bound, which every case can see, and the score is
1.000 with ZERO declared equivalent.

The generalisation, now with two measurements behind it: **before declaring a
survivor equivalent, ask whether any input can make the quantity it perturbs
change an output.** If not, the site is not equivalent, it is unobservable, and
the fix is to delete the restatement. Unit #1 named a SIZE once; this named a
LOOP BOUND once. The cost is a departure from literal transcription, which is
only admissible with a proof written into the translation — this one has it.

**A kernel PASS *is* bit-identity for a CHARACTER output, and that is not a
contradiction of unit #3.** KGen's generated comparison for `CHARACTER(len)`,
`kv_findline_character_maxparamlength_`, is `IF (var == kgenref_var)` →
IDENTICAL, ELSE → OUT_TOL. No `rmsdiff`, no `kgen_tolerance`, no `IN_TOL`
branch. Unit #3's caveat — an ABSOLUTE RMS against 1e-14 that lets a 1e-11
relative error print PASSED — is a fact about REAL fields. The rule is
therefore: **read the generated comparison for each new element type**, and do
not carry a hedge across types where the instrument is exact.

**VIT declined to build its own kernel red test and said so.**
`NON_DISCRIMINATING — 63 case(s) compared IDENTICAL, and the kernel did not
report a mismatch when the translation was deliberately changed.` Its automatic
red test perturbs a by-value floating-point argument and this unit has none, so
it refused to claim discriminating power it had not measured. That is the tool
doing the right thing, and the line is kept in the evidence rather than argued
away; the manual no-op stub is what answers it (31 of 63 cases OUT_TOL, matching
the 31 of 63 captured strings that actually change).

**Two defects in this campaign's own scripts, both found by a run that should
have worked, both fixed by addition:**

1. `reset_to_clean.sh` restored the SOURCE and left the OBJECT instrumented.
   `vit extract` puts its own pragma-carrying file back when it finishes, so
   `ROSCO_Helpers.f90` returned textually clean, step 1 skipped it (no `_c(`
   wrapper), its mtime never moved, and make had no reason to recompile. Every
   step reported success and the script then exited 1 on its own assertion:
   `kgen symbols in libdiscon.so: 1`. It could detect the contamination and not
   repair it. Step 3c now removes any build object that DEFINES kgen symbols —
   a rule about contents, so it covers whatever the next extraction
   instruments. Red-tested in both directions on the same run: 2 removed and
   the assertion 1 → 0, then 0 removed and still 0.

2. `harness.sh --red-test` was **accepted and silently dropped in pre mode**.
   The pre path returns before the stamping block, so a pre-integration red
   test wrote `failed: 27` with nothing about what was perturbed — the script's
   own header says a red artifact that cannot say what it measured is not
   evidence about the instrument. `_harness_stamp.py` now takes `--pre`.

**NOT method, target.** The string kind, the `--perturb-to 'if (false && ...)'`
habit, and both script fixes are in the RUNBOOK's target layer.

**CANDIDATE METHOD AMENDMENT, flagged and NOT made.** The gate red test as SPEC
frames it asks whether the gate can see a *perturbation*. Three units in, that
has twice given a false negative for a reason the perturbation choice caused
rather than the unit: RUNBOOK's own attempt-1 (a line the scenarios never run)
and, less obviously, unit #4's off-by-one (a perturbation an equality test
cancels). The stronger question is whether the gate can see the unit's
*absence*. Making a unit a no-op is not always as easy as forcing one guard
false, so this is a candidate rather than a rule — but "perturb toward absence,
not toward a different answer" is a sharper statement of what the red test is
for, and it would have got the right answer on unit #4 in one attempt instead of
two. The Driver may raise it; it is not made here.

---

## Unit #5 — ExtController, 2026-08-11: `blocked`, and the first unit with no oracle

### The decision

`ExtController` closes `blocked`, with `blocked_substrate` escalated. It is not
translated. This is the first unit in the campaign that ends without a
translation, and the reasoning is worth stating plainly because "I could not
make the tool work" and "no instrument in this campaign can verify this" look
alike in a report and are not the same claim.

Four units closed before this one, three of them with a gate that could not see
them. Every one still closed, because *something* could see them: `AddToList`
was dead and the differential harness ran the clean Fortran as an oracle;
`Conv2UC` was cancelled downstream and the kernel plus the harness carried it.
The route in both cases was **P7 — the oracle is the original source.**

Here the original source **cannot be run**. That is the whole decision.

### What was measured, in the order it was measured

1. **`ExtControl.f90` is 0 of 28 executable lines in all 27 scenarios.**
   `Ext_Mode` is `0` in all 14 `Examples/*.IN` — one distinct value across every
   input file — `vit_sim.py` never patches it, and `DISCON.F90:90` sits under
   `IF (CntrPar%Ext_Mode > 0 .AND. ...)`.

   The denominator is not decoration. `scripts/coverage.py` stores only lines
   with a **non-zero** hit count, so a file that never ran and a file that was
   never instrumented are **the same empty dictionary**, and four files read as
   empty in the committed artifact. Re-running with the denominator printed is
   what turns "no entry" into `0/28`, twenty-seven times. This is the same
   failure the script's own docstring records about `gcov`'s notes file, one
   level up: there the numerator was wrong, here the numerator is right and
   unfalsifiable without its denominator.

2. **The gate is blind to it, measured against the Release build, twice.**
   Perturbing Record 49 moved 0 of 5,252,000; making the unit a **no-op**
   (first statement → `RETURN`) moved 0 of 5,252,000. Both artifacts record
   `replacements: 1` and `revert_verified: true`. The second is the form the
   RUNBOOK now prescribes — perturb toward absence — and it is what makes the
   claim about the *unit* rather than about the perturbation.

3. **Forcing the oracle to run makes it segfault.** `Ext_Mode = 1` through
   `vit_sim.py`'s own `write_discon(patches=...)`: **exit status −11, signal
   11**. `DLL_FileName` is the literal string `"unused"` in all 14 inputs, no
   external Bladed-style library is shipped anywhere in the tree, and
   `ExtController` never checks `ErrVar%ErrStat` after `LoadDynamicLib` — so
   `dlopen` fails, `ProcAddr(1)` stays `C_NULL_FUNPTR`, the code prints
   *"Library loaded successfully"*, and the next two statements are
   `C_F_PROCPOINTER` on that null and a CALL through it.

   This was measured rather than reasoned about, and the order matters: the
   source was read first and the probe written to test what the source
   predicted. Reasoning alone would have got the same answer here and has been
   wrong before in this campaign.

4. **VIT refuses to generate anything, deterministically and correctly.**
   `TYPE(ErrorVariables)` has four fields and exactly one blocks it:
   `CHARACTER(:), ALLOCATABLE :: ErrMsg`. Not a defect — a refusal with a
   correct message.

### Why the substrate gap was NOT closed here, when unit #1 and unit #4 closed theirs

Unit #1 built the descriptor bridge; unit #4 built the harness's string kind.
Both were the right call and the same argument applies to `CHARACTER(:),
ALLOCATABLE`: **37 of the 69 units take a `TYPE(ErrorVariables)` dummy**, so it
is on the critical path and will have to be built.

The difference is that building it **would not close this unit.** Even with a
bridge there would be nothing to compare the translation against. Spending the
unit's budget on a feature that leaves the unit exactly as unverifiable would
have produced a translation with no evidence — which is the failure mode this
whole method exists to prevent, arrived at by working hard rather than by
cutting corners. The gap is escalated with its blast radius measured, which is
what the Driver needs in order to schedule it against the other 36 units.

`vit analyze-types --fix character` — the remedy VIT's own error message
suggests — is **refused**. It rewrites ROSCO's type definition to a fixed
length, changing `LEN(ErrVar%ErrMsg)`, which sizes `avcMSG`, which is Record 49.
Changing the oracle to make the translation checkable inverts P7.

### Two defects, each recorded before it was fixed (C12)

1. **VIT emitted a five-argument wrapper over a three-argument bridge, in
   silence.** With no view strategy configured, `vit interface` produced
   `CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(ExtDLL_view))` —
   `LocalVar` and `ErrVar` declared in the wrapper's dummy list and absent from
   the call, with no diagnostic anywhere in the output. `LocalVar%iStatus` is
   the guard on the entire initialisation branch, so that bridge could never
   have loaded the library it exists to call. It compiles and it links.

   Same shape as unit #1's dropped ALLOCATABLE attribute, one level up: there an
   attribute went missing, here two whole arguments. The wrong artifact is kept.

   Fixed in VIT (`f8ab74f`), not worked around (X2). `dropped_derived_args`
   already existed and only `test_validate` asked it, so the warning now lives
   in `generate_fortran_wrapper` — every path that ships a wrapper goes through
   it, and a fourth caller added later cannot forget to ask. Additive: no
   generated byte changes, so no artifact already measured against an earlier
   revision is invalidated. Red-tested both directions; 937 tests pass (935 + 2).

   **The habit did not save this; reading the wrapper did.** RUNBOOK's unit #1
   entry says to read the emitted wrapper attribute by attribute whatever the
   tool's current answer is. That is what caught it. A habit is weaker than a
   tool, which is why the tool changed.

2. **A crashed run can reconfigure the gate where no clean-tree check can see
   it.** This probe's first draft restored `Examples/DISCON.IN` in a `finally`,
   and **a `finally` does not run through SIGSEGV**. The run left `Ext_Mode = 1`
   in the gate's own input — and `Examples/DISCON.IN` is **gitignored**
   (`.gitignore:78`), so `git status` stayed clean and `done.py`'s P2 could not
   have caught it. Every gate run afterwards would have measured a silently
   reconfigured controller.

   The general form, and it is not specific to this probe: **a restore that must
   survive a crash belongs in a parent process, not in the process that
   crashes.** `gate.py` snapshots and restores these files and is the reason the
   campaign has not been bitten before; nothing else did.

### On the disposition, and on `done_check.py`'s verdict

`done_check.py` will report `ExtController` INCOMPLETE: P9 (gate) passes on a
vacuous green, and **P11 and P12 FAIL** because `harness/ExtController.json` and
`mutation/ExtController.json` do not exist. **That verdict is correct and is
kept.** The RUNBOOK says a unit that is not COMPLETE is not finished — and this
unit is not finished; it is *blocked*, which is a different claim, and P8 exists
so that the distinction can be recorded instead of halting the run. Manufacturing
a harness artifact for a unit with no bridge and no oracle would have made the
condition green and the campaign's evidence weaker, silently — the same trade
the RUNBOOK refuses when it forbids unsetting `--mutation-glob`.

The absence of those two files is therefore load-bearing evidence, and it is
named as such in `plan.json`'s `observability` rather than left to be noticed.

### NOT method, target

The coverage-denominator gap, the gitignored-input hazard, the `CHARACTER(:)`
refusal and the `ExtDLL_Type` bridge are all in the RUNBOOK's target layer.

**CANDIDATE METHOD AMENDMENT, flagged and NOT made.** SPEC §P8 lists five
dispositions and says each must state the evidence it rests on. Four units of
evidence here suggest a missing distinction *within* `blocked`: this unit is
blocked because **no oracle exists**, which is a permanent property of the
campaign's fixtures until someone adds one, and is categorically different from
blocked-on-a-tool-gap (unit #1, which was answered in two dispatches by building
the feature). A Driver reading `blocked` alone cannot tell which it has, and the
two schedule completely differently — one is engineering work with a known
endpoint, the other is a decision about what the campaign's inputs are for. The
`evidence` field carries the difference today, in prose. Whether the vocabulary
should carry it is the Driver's call; it is not made here.

**A SECOND CANDIDATE, weaker, also flagged.** SPEC §10.3 says "a function no
instrument can turn red is unverified regardless of its recorded disposition."
That is exactly this unit, and the spec has no name for it. `integrated_
unexercised` is close and wrong — it claims translation and verification, and
`AddToList` shows the campaign already uses plain `integrated` for a verified-
but-unexercised unit. The gap is a name for *verified by nothing, and here is
why*. Raised, not resolved.

## Unit #5 — ExtController, second dispatch, 2026-08-11: the blocking claim was wrong, and two substrate gaps are closed

### The decision

The first dispatch closed this unit `blocked` on an argument with two legs.
**One leg is refuted by measurement. The other has been removed by building the
thing it said had to be built.** The disposition stays `blocked` — P11 and P12
still cannot be produced — but for a *third* reason, which is smaller, named,
and ordinary engineering rather than an impossibility.

The first dispatch's own words, from `plan.json`:

> THERE IS NO RUNNABLE ORACLE. […] So P7 — the oracle is the original source —
> has nothing to point at. […] a unit whose reference implementation cannot be
> executed to completion on any input the campaign possesses.

The last clause is the one that is true, and the one that was over-read. It
*possessed* no such input. It does now.

### 1. The oracle runs

`ExtController` crashed because `dlopen` failed, not because the function is
unrunnable. `DLL_FileName` is the literal string `"unused"` in all 14
`Examples/*.IN`; no external Bladed-style library was shipped anywhere in the
tree; `ExtController` does not check `ErrVar%ErrStat` after `LoadDynamicLib`, so
`ProcAddr(1)` stays `C_NULL_FUNPTR` and the CALL two statements later is the
signal. Every step of that chain is a fact about the INPUT.

`fixtures/bladed_stub/discon_stub.c` is 60 lines of C exporting `DISCON` with the
signature `ExtControl.f90`'s own `ABSTRACT INTERFACE` declares.
`probe_ext_mode_1_with_oracle.json`:

```
exit_status: 0    returned_normally: true    discon_in_restored_by_parent: true
```

Three properties are load-bearing and are argued in the fixture's own header:
deterministic (both sides of a differential comparison call it), dependent on its
inputs (a stub writing constants would let a translation that never fills
`ExtDLL%avrSWAP` pass — unit #2's all-zero-window vacuity again), and stateless.

**It is an addition, not a verification-default change, and getting that wrong is
what made this a Driver question.** The first dispatch escalated "construct an
oracle" under SPEC §8.4 on the grounds that it "adds a gate scenario and so
changes the gate's compared count and its baseline set". That is true of a gate
scenario and false of this: the **differential harness does not run scenarios**,
it calls the unit directly. The 27 scenarios keep `Ext_Mode = 0`, no baseline
moves, and `gate/__gate__.json`'s pinned `compared = 5252000` is untouched. P5
permits it outright. The general form is in the RUNBOOK: before escalating "an
oracle must be constructed", ask *which instrument* needs it.

The probe is a new file beside the old one rather than an edit of it. The SIGSEGV
result is still true of the campaign's own inputs and both artifacts stand.

### 2. `CHARACTER(:), ALLOCATABLE` crosses — VIT `a2e2c30`

The first dispatch declined to close this gap with an argument worth quoting,
because it is the argument that has to be rejected:

> The difference is that building it **would not close this unit.** Even with a
> bridge there would be nothing to compare the translation against.

The premise was §1, and §1 is false. With an oracle, the bridge is exactly what
is needed. The gap was already known to block **37 of 69 units**; declining it
cost two dispatches.

A deferred-length CHARACTER has no C type because its LENGTH is part of its
allocation. It crosses as three members — a staging buffer the populator module
owns, the current length, and the writable **capacity** — with the reallocating
assignment done by the reverse copy. The capacity is what makes the field
*writable* rather than merely readable, and without it
`ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)` could not be translated
at all. An over-long assignment is **refused and reported, never truncated**: a
shortened error message is a plausible wrong answer, which is the one kind of
wrong answer a bit-for-bit comparison cannot catch.

`vit analyze-types --fix character` is still refused for the same P7 reason and
is now also unnecessary.

**A defect inside the fix, found by RUNNING the generated code, not reading it.**
The populator's first draft filled the buffer with `buf = src%ErrMsg` — a
whole-variable assignment to a deferred-length ALLOCATABLE, so Fortran 2003
automatic reallocation applies and the buffer comes straight back out at
`LEN(src%ErrMsg)`, discarding the headroom allocated one line above and making
the capacity published to C++ **a lie in the dangerous direction**. Every shape
assertion passed. The round-trip fixture reported `cap=5` where 4101 had been
allocated. That is the whole argument for
`tests/test_deferred_char_roundtrip.py`, which compiles and runs the generated
bridge; red-tested in both directions.

### 3. A generated bridge no compiler could read — VIT `83d25f9` (C12)

Found on the way and **larger than this unit**. `vit test-validate` emitted the
differential bridge's dummy list on one line. Decomposing `ControlParameters`
(214 fields) and `LocalVariables` (168) produced a `SUBROUTINE` statement
**11,747 characters long**; free-form Fortran stops at 132 columns, so gfortran
truncated it and reported **1,153 diagnostics, none of which name the cause**.

**Every unit in this campaign taking either type — most of ROSCO's controllers —
was outside `vit test-validate` entirely, and nothing said so.** P11 and P12 are
mandatory for all of them. The first four units took scalars, arrays and strings
and never touched a derived type, which is the only reason this went unmeasured.

Fixed as a rule about line *contents* rather than a list of emission sites,
because wrapping the dummy list alone left the array copy-in statements at
133–153 columns. Two defects in the fix itself, both caught by testing rather
than reading: an offset-tracking loop that never terminated and **hung the test
suite**, and a test that asserted the wrong property and failed against a correct
implementation. `statemachine_bridge.f90` goes from 1,153 errors to 0.

### 4. What still blocks it, and why the disposition is still `blocked`

`harness/ExtController.postintegration.json` and `mutation/ExtController.json`
still do not exist, so P11 and P12 still FAIL and `done_check.py` is still right.
Manufacturing either remains the trade this campaign refuses. What changed is the
*reason*, and there are three, all named in `plan.json`'s escalation:

* **A.** `vit test-validate` must cross a deferred-length CHARACTER **field** of
  a decomposed type. Same three-member shape as §2, one generator over; same 37
  of 69 units.
* **B.** `harness/vitbridge.py`'s `expand_derived` drops every CHARACTER field —
  its own comment says so — so VIT's bridge would declare parameters the case
  generator does not produce. Unit #4 built the `char[]` kind for CHARACTER
  *arguments*; this is fields.
* **C.** *A judgement, not a feature.* Every generated case must supply a
  **loadable** `DLL_FileName` or the reference crashes, and the SAVE `DLL_Ext`
  means case ORDER matters too. That is pinning a field to a fixture path and
  constraining ordering, in a generator whose stated premise is that narrowing
  the input domain is the blindness it exists to remove. It is defensible — the
  admissible domain really is that narrow, because the original crashes on the
  rest of it, which is itself a finding about upstream ROSCO — but it is a
  decision about what the harness may hold constant and the artifact must report
  it. **Raised, not made.**

Also unmeasured and cheaper: the `TYPE(ExtDLL_Type)` production bridge for
`LoadDynamicLib` and its SAVE state (X1 forbids inlining), and whether
`ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))` is auto-extracted by
`vit/allocate_extract.py`.

### 5. On the first dispatch's candidate method amendment

It proposed splitting `blocked` into blocked-on-a-tool-gap and blocked-because-
no-oracle-exists, on the grounds that the second is "a permanent property of the
campaign's fixtures until someone adds one".

**This dispatch is evidence against that amendment**, and it is worth saying so
because the amendment was raised from this unit. The distinction it draws did
not survive contact: what looked like a permanent property of the fixtures was
60 lines of C. A vocabulary that had let this unit be filed as categorically
different would have made the wrong answer easier to keep. The existing
`evidence` field carried the difference in prose, and prose was reviewable.

The second candidate — a name for *verified by nothing, and here is why* — is not
affected and still stands.

### A defect in this session's own gitignore hygiene

`.gitignore:65` is `*build*`, which silently matched
`fixtures/bladed_stub/build.sh`. The first commit of the fixture added the `.c`
and left the script untracked, so a committed evidence file named a library
nothing committed could produce (K3) — and `git status` was clean throughout.
Same shape as the gitignored `Examples/DISCON.IN` the first dispatch found. After
committing a new fixture, `git check-ignore -v` each file rather than trusting a
clean status.

### NOT method, target

All of it. The oracle-vs-instrument distinction, the 132-column limit, the
deferred-CHARACTER shape and the gitignore hazard are RUNBOOK target-layer
entries. No amendment to the invariant layer is proposed; the one the first
dispatch proposed is argued against above.

## Unit #5, third dispatch — 2026-08-11: `ExtController` is `integrated`

### 1. The blocking claim was wrong for the second time, and for a smaller reason

The second dispatch closed `blocked` on three remaining items and called them
"ordinary engineering with a known endpoint". They were, and they are done. What
that dispatch could not have known is that doing them would surface a defect
larger than the unit — see §3.

The pattern across three dispatches is worth naming because it is not about this
function. Each dispatch's blocker dissolved when someone asked **which instrument
needs this**, rather than **is this unit verifiable**:

* #1 escalated "an oracle must be constructed" to the Driver under SPEC §8.4,
  on the assumption that an oracle meant a gate scenario. The differential
  harness does not run scenarios. It was P5, and 60 lines of C.
* #2 declined to close the `CHARACTER(:)` gap on the grounds that it "would not
  close this unit". The premise was #1's, and #1 was false.
* #3 found that the differential bridge had been discarding outputs for the
  whole campaign. No amount of reasoning about `ExtController` would have shown
  that; only running the harness did.

### 2. Gap C is a JUDGEMENT and it is now made, in a file that can be reviewed

`harness/ranges.toml` did not exist in this campaign. It does now, with four
pins for this unit and a header saying what the file is for: **the only
judgement in the pipeline**, and every entry NARROWS the generated domain, which
is the blindness the generator exists to remove.

The four are `CntrPar_DLL_FileName` (the fixture path), `CntrPar_DLL_ProcName`
(`DISCON`), `LocalVar_iStatus` (0) and `ExtDLL_avrSWAP_n` (−1, meaning
unallocated). **Every one exists because the REFERENCE crashes on the rest of
the domain, not because the translation would**: `ExtController` never checks
`ErrVar%ErrStat` after `LoadDynamicLib` and then calls through
`DLL_Ext%ProcAddr(1)`, which is `C_NULL_FUNPTR` whenever the load failed. That
is a fact about upstream ROSCO and the file records it as one.

What the pins COST is stated in `plan.json`'s `observability` rather than left
to be discovered: the `iStatus /= 0` path is never exercised, two of the three
DLL strings are held at one value each, and `LEN_TRIM(ErrVar%ErrMsg)` is 0 in
every admissible case. The third of those is where two of the four
declared-equivalent mutants come from.

`statevary.constrain` gained `text = "..."` so a pinned string is written as the
string it is. A judgement nobody can read is not one anybody can review.

### 3. C12 — the decomposed bridge was discarding outputs, for the whole campaign

Recorded before it was fixed; the wrong artifact is
`evidence/ExtController/vit_defects/extcontroller_bridge.alloc_input_only.f90`.

`vit test-validate` decomposes a derived-type argument into one bridge parameter
per field. It copied every field IN. It copied back only fixed-size arrays,
nested types and fixed-width CHARACTERs. A **rank-1 ALLOCATABLE** field and a
**SCALAR** field of an INOUT argument were input-only:

* the reference wrote into a Fortran allocation the bridge then deallocated,
  while the translation wrote into the caller's buffer — so the two were never
  compared on that field at all;
* a scalar crossed by VALUE, so the harness compared its own INPUT against the
  translation's answer. On `ExtController` that reported the translation wrong
  on all 163 cases, which is the loud version; a unit whose scalar output
  happens to equal its input would have reported a full green.

`ExtDLL%avrSWAP` is everything `ExtController` produces and `ErrVar%ErrStat` is
where ROSCO puts every error status. Both were invisible.

Fixed in VIT `8c34ceb`. The extent goes BY REFERENCE, because the callee chooses
it — `ExtController` receives the field unallocated and returns 2000 elements —
and a CAPACITY says how much room the caller has, with three states kept
distinguishable because one of them is absence (P6): `n < 0` unallocated,
`n == 0` allocated-empty, `n > 0` allocated. A result past the capacity is
REFUSED and reported, never truncated, which is the same rule the view struct's
reverse copy already used.

**The proof that it matters is the red test**: with the translation replaced by
a no-op the harness fails 163 of 163 and names `ExtDLL.avrSWAP`,
`ExtDLL.n_avrSWAP`, `ErrVar.ErrStat` and `ErrVar.ErrMsg` — the four outputs that
no harness in this campaign could see before this commit.

### 4. Where the blindness is in the INSTRUMENT, change the instrument

The mutation score went 0.758 → 0.904 → 1.000, and each step is a different kind
of move:

* **0.758 → 0.904, by REMOVING restatements.** `accINFILE`'s bound and record 50
  are the same expression, and so are `avcOUTNAME`'s bound and record 51.
  Transcribing each twice leaves a second site computing a quantity nothing
  reads back. Naming each once leaves only the observable site. That is the
  third time in this campaign — unit #1 named a SIZE once, unit #4 a LOOP BOUND,
  this a BUFFER LENGTH — and the test is still the one the RUNBOOK states: *can
  any input make this quantity change an output?*
* **A survivor fixed in the ORACLE, not in the translation and not by
  declaration.** `int aviFAIL = 0;` was write-only: the local is handed to the
  external library and never read again. The stub now reports the INCOMING value
  back in record 44 before overwriting it, and the mutant dies on all 163 cases.
  The fixture had already been extended once for the same reason — the bytes of
  `accINFILE` and `avcOUTNAME` were reaching the library and never being read.
  **A stub that answers without reading its inputs is unit #2's all-zero kernel
  window in another costume.**
* **0.904 → 1.000, by DECLARING four, each applied by hand and re-run first.**
  Two are one fact: `LoadDynamicLib` blank-fills `ErrMsg` through a
  `CHARACTER(*), INTENT(OUT)` dummy, so `LEN_TRIM` is 0 in every admissible case
  and the concatenation's right operand is constant. One is a capacity boundary
  four orders of magnitude out of reach. One is a buffer length nothing reads.
  The distinction this campaign preserves — REMOVED vs DECLARED — is why the
  file has four entries and not eight.

### 5. Two things about the ORIGINAL that had to be measured, not read

Both were measured before any C++ was written, and both are transcribed rather
than corrected, because the original is the oracle (P7).

* **`ExtDLL%avrSWAP(49) = LEN(avcMSG) + 1` is the constant 2.** `avcMSG` is an
  ARRAY of `CHARACTER(KIND=C_CHAR)`, whose ELEMENT length is 1; `LEN` of a
  CHARACTER array is the element length, not the array size. The record's own
  comment calls it "Maximum number of characters in the MESSAGE argument". It is
  not. `evidence/ExtController/len_probe.txt`.
* **`avcMSG = TRANSFER(C_NULL_CHAR, avcMSG)` writes ONE byte.** `TRANSFER` with
  an array MOLD returns an array just large enough for SOURCE, and gfortran
  assigns a nonconforming right-hand side by copying min(size) elements. Bytes
  2..N of an automatic array are left INDETERMINATE.
  `evidence/ExtController/transfer_probe.f90` prints `0 90 90 ...`. The
  translation zero-initialises instead — a stated departure with its proof in
  the file — and the oracle fixture deliberately does NOT fold those bytes into
  its answer, because a stub reading indeterminate memory would make the two
  sides of a differential comparison disagree for a reason about neither
  implementation.

### 6. `--auto-allocate` is unusable here, and that is now the campaign's problem

RAISED, not worked around in silence. Artifact:
`evidence/ExtController/vit_defects/integrate_auto_allocate.wrapper.f90`.

Three defects, each measured on this unit:

1. Its copy-call scan matches **any** `CALL x(..., arg%field, ...)`. Its own
   docstring says "Registry Copy calls"; the implementation says any call. It
   hoisted BOTH `LoadDynamicLib` and the external DLL call into the wrapper,
   which would have called each a second time.
2. `ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))` is classified `local` because the
   size is a local PARAMETER, so it is declined even after (1).
3. Its error handling emits `SetErrStat(...)` and `CHARACTER(ErrMsgLen)` —
   OpenFAST names that exist nowhere in ROSCO. The generated wrapper cannot
   compile on this codebase at all, so this path has never been exercised
   outside OpenFAST.

This unit needs exactly one ALLOCATE and it is in the committed wrapper, one
statement, transcribed verbatim, with all of the above beside it. **That does
not scale**: the next unit that allocates a field of an argument hits the same
wall, and a hand-edited generated wrapper is a thing that drifts. Fix belongs in
VIT (X2).

### 7. Four smaller defects in the loop's own harness, all found by one run

Each was a wrong answer in the safe direction, which is why none of them had
been seen.

1. `parse_struct_members` split on `;` **before** stripping comments, so a
   semicolon inside a `//` comment ended a member and the next real member was
   glued to the comment's tail. VIT's own generated header did exactly that and
   `n_ErrMsg_cap` was reported ABSENT FROM THE STRUCT.
2. A string's `values` are its BYTES, and two stages read them as an
   enumeration. `R2_flag_values` multiplied the case set by a 57-byte stated
   path — 218 cases became 9,635 and 211 MB of case data — and `_random_over`
   replaced the whole parameter with ONE code point drawn from its own contents,
   so the pinned library path became 1024 semicolons and the reference
   `dlopen`ed a file named `";"`.
3. `_struct_fields` decided "the C struct has no member for this extent" from
   whether the extent was FIXED, which is the same answer for the common cases
   and the wrong one as soon as an ALLOCATABLE field's extent is pinned.
4. `json.loads(run.stdout)` assumed the generated program was the only thing
   printing. `ExtController` has five `PRINT *` statements, so a clean run of
   163 cases reported "harness produced no JSON" — and `vit_mutate.py` had the
   same line, where it reported the BASELINE as a crash. Had its baseline check
   not refused to score, every mutant would have scored `killed (crash)`: a
   1.000 that measured nothing.

Two campaign scripts needed the same lesson: `_integration_shim.py` did not
include `vit_types.h` (first unit with a view-struct signature), and
`harness.sh`'s post-integration LIBS went stale the moment integration added the
four view-populator sources — repaired by CONTENTS (every object CMake built for
this target) rather than by a list, the same shape as `reset_to_clean.sh`'s
"remove any object that DEFINES kgen symbols".

### 8. NOT method, target

All of it. The oracle-vs-instrument distinction, the input-only decomposed
bridge, the `ranges.toml` judgement, the two measured facts about the original,
and the `--auto-allocate` refusal are RUNBOOK target-layer entries. No amendment
to the invariant layer is proposed.

One candidate is worth naming for the Driver and is NOT proposed as an
amendment, because the existing rules already reach it: **P3 already says a
green must be able to name what it compared and be able to go red, and this unit
is the case where the RED TEST is what proved the instrument had been discarding
outputs.** The no-op stub did not merely turn the harness red; the LIST OF
FIELDS it named is what showed four of them had never been compared before. A
red test that reports WHICH outputs moved is strictly more informative than one
that reports that some did, and this campaign's harness already does it. Nothing
to amend — worth knowing.
