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
