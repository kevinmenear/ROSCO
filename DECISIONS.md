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
