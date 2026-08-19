# Parallel scenario runs — making the gate 5× faster

**Written:** 2026-08-19, from measurement rather than reading. Every number below was taken on
this machine, in `vit-dev`, against this campaign's own baselines.

**Status:** plan — nothing here is implemented. The *proof of concept* is complete and passed.

**Where this sits:** entirely target-layer. `gate.py`, `vit_sim.py` and `Examples/` are
ROSCO-specific; nothing here belongs in `translation-loop`.

---

## Progress

| Phase | Objective | State |
|---|---|---|
| A | A worker pool replaces the serial loop in `run_scenarios` | ✅ done |
| B | Workspaces are built, reused and torn down safely | ✅ done |
| C | The gate proves it still measures the same thing | ✅ **done — 50,605 reproduced exactly** |
| D | Turn it on, with a way to turn it off | ✅ done (D.3 waived, see below) |

---

## 0. The measurement

`gate.py` runs 27 scenarios in a `for` loop, one at a time. Replacing that loop with 8 concurrent
workers, each in its own copy of `Examples/`:

| | serial | 8 workers |
|---|---|---|
| 27 scenarios | **141 s** | **28.6 s** |
| speedup | — | **4.93×** |
| outputs | — | **27/27 bit-identical** |

**5,252,000 values compared against `baseline_arrays/`, zero mismatches** — the same count the
gate itself reports, so this is the gate's full comparison, not a sample. Every scenario ran
exactly once (27 runs recorded for 27 scenarios), and no run exited non-zero.

**What that is worth per dispatch.** From `.vit/cycle_log.jsonl` for ParseDbAry_Opt's 4th
dispatch: `gate` 4 runs × 142 s, `gate_redtest` 12 runs × 249 s = **59.2 min**, and 97% of it is
scenario execution (a full controller rebuild measures 3.3 s, not minutes).

```
gate + redtest   59.2 min  ->  ~13 min
dispatch         176 min   ->  ~130 min      ~46 min saved, 26%, about 1.35x
```

**The long pole is scenario 1 at 22 s.** With 27 tasks summing 141 s over 8 workers the floor is
`max(141/8, 22) = 22 s`; measured 28.6 s. The gap is scheduling — s1 did not start first.
**Longest-first ordering should recover most of it** (~22-24 s, ~6×), and more than 8 workers buys
nothing.

---

## 1. What a scenario actually writes

Measured by diffing a worked workspace against a pristine one, not by grepping:

| written | covered today by |
|---|---|
| `DISCON.IN`, `DISCON_awc.IN`, `DISCON_flp.IN`, `DISCON_ol_mode2.IN`, `DISCON_su_sd_tra.IN` | `snapshot_inputs` / `restore_inputs` |
| `vit_sim<N>.RO.dbg` | nothing — gitignored (`Examples/.gitignore:8`), accumulates as disk debris |

Reads only: `Tune_Cases/`, `Test_Cases/`, `examples_out/` (including the pickled turbine model
`01_NREL5MW_saved.p` — **not** debris, despite living in a directory that is mostly debris).

**This is why the gate needs `snapshot_inputs` at all**, and the campaign has already been bitten:

> *"the gitignored `Examples/DISCON.IN` that let a crashed probe reconfigure the gate where no
> clean-tree check could see it"* — `.gitignore:164`

---

## 2. Per-worker copies, NOT a parameterised input path

Two ways to isolate. The plan takes the first, and the reason matters.

**Copy `Examples/` per worker.** `vit_sim.py` derives `this_dir` from its own location, so a copy
*is* the isolation — no code change at all. Measured: **8 workspaces, 411 MB, built in seconds**,
with `Tune_Cases`/`Test_Cases`/`examples_out` symlinked (read-only) and `vit_sim.py` + `*.IN`
copied (rewritten per scenario). Hard links do not work — `/workspace` is a bind mount and `/tmp`
is overlay, so they are cross-device.

**Parameterise `this_dir`** so workers share one directory. Rejected:

- `vit_sim.py` is declared `intent = "copy"` in `campaigns/rosco-r2.toml`, byte-identical to the
  first replication's. Changing it means redeclaring to `derive` **and red-testing it** — a real
  provenance cost, for a change the copy approach does not need.
- More importantly: with per-worker copies the campaign's own `Examples/` is **never written at
  all**. That is strictly better than snapshot-and-restore, because there is no window in which a
  crash leaves `DISCON.IN` modified — which is the incident quoted in §1.

**Nothing to restore beats restoring.** Same principle either way.

---

## Phase A — a worker pool inside `run_scenarios`

> **Objective:** the gate runs its scenarios concurrently and returns exactly what it returns
> today.

`run_scenarios` is the **single change point**: one function, called only from `gate_once`, which
has three call sites (plain, perturbed, revert). Nothing else runs a scenario.

### Tasks

- [x] **A.1 — Replace the loop with a bounded pool**
  - [x] Width is a parameter, defaulting to the container's CPU count (8 today)
  - [x] Preserve the return contract exactly: the list of scenarios that exited non-zero
  - [x] Each scenario stays its own process — the DLL holds Fortran `SAVE` state that
        `dlclose()` does not reliably release (`gate.py:52`), so this is required, not incidental
- [x] **A.2 — Longest-first ordering**
  - [x] Start s1 (22 s) first; measured makespan should fall from 28.6 s toward ~22 s
  - [x] `SCENARIO_ORDER`'s intent survives: 3/4/5 lead because they historically break first, and
        with 8 workers they are all in the first wave anyway
- [x] **A.3 — Failure attribution**
  - [x] A worker that dies must not silently drop its scenario — an unrun scenario is a missing
        `.npz`, which `compare()` already reports, but the failure list must still name it

**Done when:** a gate run reproduces its serial result and its wall clock is recorded against the
141 s baseline.

**Notes:**

>

---

## Phase B — workspaces

> **Objective:** N isolated `Examples/` copies exist when the pool runs and are gone afterwards,
> and the campaign's own `Examples/` is never written.

### Tasks

- [x] **B.1 — Build**
  - [x] Container-local (`/tmp`), never inside the campaign tree
  - [x] `vit_sim.py` and `*.IN` copied; `Tune_Cases`, `Test_Cases`, `examples_out` symlinked
  - [x] Rebuild per gate run — measured at seconds, and it removes every staleness question
- [x] **B.2 — Prove the isolation**
  - [x] After a full gate run, `Examples/*.IN` are byte-identical to before — **not restored,
        untouched.** `snapshot_inputs`'s `restored` list must come back empty.
  - [x] `residual_dirt()` still clean
- [x] **B.3 — Teardown**
  - [x] Removed on exit; a killed run leaves scratch behind, which costs disk rather than
        correctness

**Done when:** a gate run leaves the campaign's `Examples/` bit-identical to how it found it.

**Notes:**

>

---

## Phase C — prove it still measures the same thing

> **Objective:** the parallel gate can still go red, and its green means what the serial green
> meant.

### Tasks

- [x] **C.1 — Same verdict on the same input**
  - [x] A plain gate run: parallel and serial produce identical `compared` and `mismatched`
- [x] **C.2 — It can still go red**
  - [x] Re-run an existing red test (`gate/*.redtest.*.json`) under the pool and reproduce its
        recorded mismatch count **to the value**
  - [x] A green that cannot go red is not a measurement (P10)
- [x] **C.3 — Concurrency does not change a value**
  - [x] Already established once: 27/27 bit-identical, 5,252,000 values. Re-establish through
        `gate.py` itself rather than the bench harness.

**Done when:** one red test and one green run reproduce their recorded numbers exactly.

**Notes:**

>

---

## Phase D — turn it on

> **Objective:** the pool is the default, and one flag returns to the serial loop.

### Tasks

- [x] **D.1 — `--workers N`, default 8, `--workers 1` = today's behaviour exactly**
- [x] **D.2 — Record the width in the gate artifact**, so a result says how it was produced
- [~] **D.3 — Measure under real load** — WAIVED by the operator, not on an idle machine (see Risks)

**Done when:** a real dispatch's gate time is recorded and compared against the 59.2 min baseline.

**Notes:**

>

---

## Risks

- [ ] **Contention with the agent.** Every number here was taken on an **idle** machine. During a
      dispatch the agent is also working, and 8 workers may not get 8 CPUs. This is the largest
      unknown and D.3 is what closes it. If it bites, lower the width — the curve is gentle.
- [ ] **Memory.** 8 concurrent simulations in a 7.7 GB container was never measured; it did not
      fail, which is weaker than knowing the headroom.
- [ ] **Disk.** 411 MB of scratch per gate run, rebuilt each time. Cheap, but not free, and a
      killed run leaves it behind.
- [ ] **`.RO.dbg` accumulation.** Already true serially — the campaign's `Examples/` holds a
      62 MB `.dbg2`. Per-worker copies move this into scratch, which is an improvement, but it is
      worth noticing rather than discovering.
- [ ] **A silent partial run.** If the pool drops a scenario, the `.npz` is missing and
      `compare()` reports it — but that path deserves a deliberate test rather than trust.

**Notes:**

>

---

## Deliberately out of scope

- **Parallelising anything other than scenarios.** The controller build is 3.3 s; `-j8` would
  save ~17 s across a whole dispatch. Measured, and not worth doing.
- **Changing `vit_sim.py`.** §2. The copy approach needs no change, and the manifest
  redeclaration it would force is a cost with no benefit here.
- **The agent's own time.** It is 50-94% of a dispatch and nothing here touches it. This plan
  addresses the 34% that is gate, and takes about 26% off the total.


---

## Results — implemented 2026-08-19

Measured **through `gate.py` itself**, not through a bench harness.

| run | serial | 8 workers | speedup |
|---|---|---|---|
| plain gate | 148 s | **27.2 s** | **5.4×** |
| `ActiveWakeControl` D2R red test | 249 s (recorded) | **55.5 s** | **4.5×** |

**C.2 — the acceptance gate, reproduced to the digit.** Every field of the recorded serial
artifact matched:

| field | recorded (serial) | now (8 workers) |
|---|---|---|
| compared | 5,252,000 | 5,252,000 |
| **mismatched** | **50,605** | **50,605** |
| revert_compared | 5,252,000 | 5,252,000 |
| revert_mismatched | 0 | 0 |
| revert_verified | true | true |
| scenarios_failed | [] | [] |

The gate still goes red on the same perturbation by the same amount, and comes back green on
revert. That is the whole claim.

**B.2 — the isolation, demonstrated rather than argued.** `Examples/DISCON.IN` and
`DISCON_awc.IN` are **byte-identical** before and after, and `inputs_restored` came back **empty**
— nothing was written, so nothing needed restoring. `residual_dirt` empty. Worker scratch removed.

**`--workers 1` is the original code.** `diff` of the serial branch against the pre-change loop
differs by exactly one line: the `if workers <= 1:` guard. The fallback was run and passed
(148 s, 5,252,000 compared, 0 mismatched).

**D.3 waived by the operator** on the reasoning that a pool degenerates to serial under
contention, so the downside is "less than 5×", not "wrong". The correctness half (Phase C) was
**not** waived and is what the 50,605 establishes. The recorded `scenario_workers` field means any
future artifact says how it was produced, so the first real dispatch will price it for free.
