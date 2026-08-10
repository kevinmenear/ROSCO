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
