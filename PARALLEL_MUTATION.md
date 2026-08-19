# Parallel mutation sweeps — making the sweep ~7× faster

**Written:** 2026-08-19, from measurement rather than reading. Every number below was taken on
this machine, in `vit-dev`, against this campaign's own ParseInAry_Opt harness.

**Status:** plan — nothing here is implemented. The *proof of concept* is complete and passed.

**Where this sits — and it is NOT where `PARALLEL_SCENARIOS.md` sat.** That work touched
`gate.py`, `vit_sim.py` and `Examples/`, all ROSCO-specific target layer. This work touches
`scripts/vit_mutate.py`, which lives in **`translation-loop` — the invariant layer, shared by
every future campaign**. Three consequences that did not apply last time:

- it needs tests in `translation-loop/tests/`, and any new file must be named in `PLAN.md` §2.6
  or `test_layout.py` goes red (this has already cost two commits once);
- a mistake here is inherited by every campaign that follows, not just this one;
- the serial path must survive as the documented way back, exactly as `--workers 1` did for the
  gate.

---

## Progress

| Phase | Objective | State |
|---|---|---|
| P | The harness can link at all (prerequisite, not parallelism) | ✅ **done — baseline 19,536 checked, 0 failed** |
| A | A worker pool replaces the serial `for` loop over mutants | ✅ done |
| B | Per-worker workspaces are built, reused and torn down | ✅ done |
| C | **Mutant identity** — each worker provably scored the mutant it was given | ✅ **done — red-tested** |
| D | **Watchdog under load** — no false kills from concurrency | ✅ done |
| E | The sweep proves it still measures the same thing | ✅ **done — 153/153 identical, twice** |
| F | Turn it on, with a way to turn it off | 🟡 on; F.6 needs a real dispatch, F.7 open |

---

## 0. The measurement

24 full mutant cycles — write the mutant, rebuild under `-fsanitize=address,undefined`, run the
whole case corpus — each worker in its own workspace under container-local `/tmp`:

| workers | 24 cycles | per cycle | speedup | efficiency | peak RAM |
|---|---|---|---|---|---|
| 1 | 143.8 s | 5,992 ms | 1.00× | — | 1,137 MB |
| 4 | 38.4 s | 1,600 ms | 3.74× | 94% | 2,367 MB |
| 8 | **20.7 s** | **864 ms** | **6.93×** | **87%** | 3,687 MB |

**Memory is not the constraint.** Peak was 3.7 GB of the container's 7 GB at full 8-way
concurrency, leaving 3.3 GB of headroom. ASan's shadow mapping is virtual; it never bound. The
container does not need more memory allocated to it.

**The cycle is half build, half run, and both halves parallelize:**

```
build (g++ -fsanitize=address,undefined, 1 TU + link of 74 objects)   2,899 ms
run   (whole case corpus under ASan/UBSan)                            3,066 ms
```

That is why efficiency holds at 87% out to 8 workers — there is no serial fraction of any size
to bound it.

**What that is worth per dispatch.** From `.vit/cycle_log.jsonl`, ParseInAry_Opt's 6th dispatch:

```
mutate         23 runs   79.9 min   62% of a 129-minute dispatch
gate_redtest    4 runs    3.1 min
gate            4 runs    1.7 min
                          ------
instrumented             84.8 min   66% of wall
```

```
sweep    79.9 min  ->  ~11.5 min at 8 workers      ~68 min saved
```

**Read the 62% honestly.** This was a re-dispatch of a unit failing on mutation score, so it was
mutation-heavy by construction; a first dispatch does more translate/verify and less sweeping.
62% is the shape of a mutation-bound dispatch, not a campaign-wide average. The *ratio* (6.93×)
is what the projection rests on, and that was measured directly.

**A discrepancy to keep in view.** This campaign's own `merged_why` estimated ~14 s per mutant;
I measured 6.0 s. My cycle may not carry the per-case stdout capture that `compare_record`
added. That moves the absolute minutes, not the speedup.

**The second prize, which may be worth more than the minutes.** The 600 s foreground ceiling is
what forced this unit's 153 mutants into eight separate `sanpart` files plus `_mutation_merge.py`
(`merged_why` says so in as many words). At 8 workers the same 153 mutants land near 130 s —
one part, no slicing arithmetic, no merge, and no `mutant_slice` bookkeeping to get wrong.

---

## 1. What one mutant cycle actually writes

Read out of `vit_mutate.py:571` and the harness `Makefile`, not assumed:

```python
cpp.write_text(original[:m.pos] + m.after + original[m.pos + len(m.before):])
build_and_run(out_dir, ...)     # rm {stem}.hpp, {stem}_test.o, test; make; ./test cases
```

```make
parseinary_opt.hpp: /workspace/.../parseinary_opt.cpp
	cp $< $@
test: parseinary_opt_test.o parseinary_opt_bridge.o parseinary_opt_callees.o
	$(CXX) -o $@ $^ $(LIBS)
```

| written per mutant | shared today | must become per-worker |
|---|---|---|
| the translation `.cpp` | **yes — the campaign's real source file** | ✅ yes |
| `{stem}.hpp` (a copy of it) | yes | ✅ yes |
| `{stem}_test.o` | yes | ✅ yes |
| `test` binary | yes | ✅ yes |
| whatever the run writes in `cwd=out_dir` | yes | ✅ yes |
| the ~74 objects in `LIBS` | yes | ❌ no — read-only, keep shared |
| `{stem}_bridge.o`, `{stem}_callees.o` | yes | ❌ no — built once, never rebuilt |

**This is the key structural fact.** The heavy inputs — 74 prebuilt production objects reached by
absolute path — are read-only for the whole sweep. The per-worker payload is one `.cpp`, one
Makefile, two small `.o` files, the test source, and a symlink to the case corpus. A few MB. That
is *lighter* than the `Examples/` copy that made the gate work.

**Notes:**

---

## 2. Why per-worker copies, and not a lock

A lock around the mutate-build-run cycle serialises the entire thing, which is the whole cost.
There is nothing left to run in parallel. The copy is the isolation, same as the gate.

The one wrinkle the gate did not have: the harness `Makefile` reaches the translation by
**absolute path**, so a copied workspace still points at the shared original. One `sed` on the
copied Makefile repoints it at that worker's own mutant:

```
sed -i "s|^{stem}.hpp: .*|{stem}.hpp: $D/src.cpp|" $D/Makefile
```

Verified in the proof of concept — this is exactly how the 24-cycle measurement above ran.

**Notes:**

---

## 3. The safety win that comes free

`vit_mutate.py` mutates the campaign's real translation in place and restores it in a `finally`.
Killed, it does not restore. `mutate_guarded.sh` exists solely because **that has now happened on
three of three hard kills** — the most recent left `VS_ControlMode < 0` reading `< 1`, one
`restore_integrated.sh` away from being compiled into `libdiscon.so` and gated as the
translation.

With per-worker copies the sweep **never writes the real `.cpp` at all**. The hazard does not get
mitigated; it stops existing. `mutate_guarded.sh` should stay regardless — it costs nothing and
guards the serial path — but it stops being the only thing between a hard kill and a live mutant.

**Notes:**

---

## 4. The two ways this can silently produce a wrong score

**This is the difference from the gate, and it is the reason for phases C and D.** Gate scenarios
only *read* what they shared, so broken isolation produced a slow run. Here the parallel unit
*writes* what it shares, so broken isolation produces **a wrong score that looks fine**.

### 4.1 Identity — did this worker score the mutant it was given?

If worker A's `make` picks up worker B's mutant, A reports *its own* mutant killed having never
compiled it. The score goes up. Nothing looks wrong. No existing check catches this: the
artifact records the mutant id, and the id is content-derived from the intended text, not from
what was built.

**The check:** after the build, hash the worker's `{stem}.hpp` and compare against the intended
mutant text. Mismatch is a hard failure of the run, not a warning. Cheap — one hash per mutant
against a build that costs 2.9 s.

### 4.2 The watchdog — concurrency manufactures false kills

```python
run_timeout = max(60.0, 20.0 * baseline_seconds)
```

Self-calibrating from a baseline taken on an **unloaded** machine. Run 8-way and every mutant is
slower, so a slow-but-honest mutant can trip the watchdog and be scored
`killed (did not terminate in Ns)` — a false kill that **inflates** the score, in the direction
that makes a unit look finished when it is not.

At 87% efficiency the per-cycle stretch measured only 15%, and the multiplier is 20×, so this is
not close today. It is written down because the margin is not guaranteed for a slower unit, a
larger corpus, or a heavier machine load, and because the failure is silent and favourable.

**The check:** any mutant that trips the watchdog is **re-run serially, alone**, before it is
allowed to count as a hang. A genuine non-terminating mutant hangs again; a mutant that was
merely crowded out passes. Also calibrate `baseline_seconds` under the same concurrency the
sweep will use, rather than before it.

**Notes:**

---

## 5. Phase P — the harness cannot link right now (prerequisite)

Found while setting up the measurement, and **it blocks the next dispatch of this unit
regardless of anything else in this document.**

ParseInAry_Opt's harness `Makefile` was generated at 15:29 with a 71-object `LIBS` list. FindLine,
GetWords and Int2LStr were integrated afterwards, so `ROSCO_Helpers.f90.o` now *references* their
`_c` bridges, and the objects that define them are not in the list:

```
undefined reference to `findline_c'   <- defined in findline.cpp.o,  absent from LIBS
undefined reference to `getwords_c'   <- defined in getwords.cpp.o,  absent from LIBS
undefined reference to `int2lstr_c'   <- defined in int2lstr.cpp.o,  absent from LIBS
```

Confirmed not to be a copy artefact: a wholesale `cp -a` of the harness directory fails
identically, with and without the sanitizer. Adding exactly those three makes it link.
`parseinary_opt.cpp.o` must **not** be added — that definition comes from the mutated header, and
adding it gives `multiple definition of ParseInAry_Opt`.

Consequence if left: every mutant returns `nocompile`, and `vit_mutate.py`'s `nocompile_ratio`
guard refuses to score rather than reporting a false number. It fails loudly, which is correct —
but it fails.

- [x] **P.1** Regenerate the harness `Makefile` via `vit test-validate`, not by hand-editing
      `LIBS`. A hand-patched list goes stale again at the next integration, which is how it got
      here.
- [x] **P.2** Confirm the regenerated `LIBS` contains `findline.cpp.o`, `getwords.cpp.o`,
      `int2lstr.cpp.o` and still excludes `parseinary_opt.cpp.o`.
- [x] **P.3** Link and run one unmutated baseline cycle; it must be green before any sweep.
- [x] **P.4** Ask whether other pending units' harnesses are stale the same way — this is a
      generic consequence of integrating a callee after a harness was generated, not a
      ParseInAry_Opt quirk.

**Notes:**

Done 2026-08-19. It was not one defect but three, each hidden behind the one before it.

1. **Regeneration without `--force` is safe.** `parseinary_opt_test.cpp` — 20 KB of hand-written
   harness, and **untracked by git, so there is no way back if it is lost** — hashed identical
   after the run. Only the `Makefile` changed. It was backed up first regardless.
2. **`vit test-validate` globbed the unit's own object into LIBS**, colliding with the definition
   the harness includes from the header. Under a sweep that is worse than a link error: the
   production object holds the UNMUTATED translation. Fixed in
   `vit/test_validate.py::_parse_link_libraries`, which now takes an exclusion set.
3. **The regenerated callee bridges recursed.** With FindLine, GetWords and Int2LStr integrated,
   their Fortran bodies are wrappers calling `<name>_c`, so a Fortran-backed bridge of the same
   name closes a loop: `bridge -> wrapper -> bridge`. It did not link wrong, it **SIGSEGV'd on a
   blown stack with empty stdout** — which reads exactly like "the harness produced nothing".
   `generate_callee_bridges` now detects the cycle *from the wrapper itself* rather than from
   `vit.yaml`'s `status:` field, because the wrapper is what actually closes the loop and a
   config field can disagree with the tree. Such a callee is left to its production object.

Regenerating drops the campaign's `LIBS += .../vit_integration_shim.o` line, which `harness.sh`
appends idempotently; it has to be re-appended after any regeneration of an integrated unit.

P.4: 26 harness directories have callee bridges; **0 would recurse today**. They were generated
before their callees were integrated, so the defect only bites on regeneration — which is
precisely what the VIT fix now prevents for every future one.

Baseline after the repair: `checked 19536, failed 0, inadmissible 0, mismatches 0`.

---

## 6. Phase A — a worker pool replaces the serial loop

- [x] **A.1** Add `--workers N` to `vit_mutate.py`, default **1**. The serial path stays the
      documented way back and must remain byte-identical to today's loop.
- [x] **A.2** Replace the `for i, m in enumerate(ms, 1)` body with a pool over a slot queue, the
      same shape `gate.py` uses (`ThreadPoolExecutor` over subprocess-bound work; the GIL is not
      in play).
- [x] **A.3** Preserve **caller-order output**. Per-mutant progress lines currently print in
      mutant order; concurrent completion will not. Either buffer and emit in order, or make the
      out-of-order-ness explicit in the line. Silent reordering makes two sweeps look different
      when they are not.
- [x] **A.4** Record `mutant_workers: N` in the artifact, beside the counts, exactly as
      `scenario_workers` was.

**Notes:**

---

## 7. Phase B — per-worker workspaces

- [x] **B.1** Build N workspaces under container-local `/tmp`: Makefile, test source, the two
      bridge `.o` files, the callees header, a **copy** of the translation, a **symlink** to the
      case corpus (8.8 MB, read-only).
- [x] **B.2** `sed` each copied Makefile's `{stem}.hpp:` rule to that worker's own `src.cpp`.
- [x] **B.3** Refuse to run, rather than falling back to serial, if the workspaces cannot be
      built — and say so on stderr. A silent fallback reports a serial sweep as a parallel one.
- [x] **B.4** Tear down on exit, including on exception.
- [x] **B.5** Confirm no worker writes anywhere under the campaign tree. Diff a worked workspace
      against a pristine one; do not grep for writes.

**Notes:**

---

## 8. Phase C — mutant identity

- [x] **C.1** After each build, hash the worker's `{stem}.hpp` and compare it against the
      intended mutant text.
- [x] **C.2** A mismatch fails the whole run. Not a warning, not a retry — the score is not
      trustworthy and must not be written.
- [x] **C.3** **Red test:** deliberately cross two workers' sources and prove the check fires.
      An identity check that has never gone red is an assumption.
- [x] **C.4** Record in the artifact that identity was verified, and for how many mutants. A
      green that cannot say what it checked is the failure mode this campaign keeps meeting.

**Notes:**

---

## 9. Phase D — watchdog under load

- [x] **D.1** Calibrate `baseline_seconds` under the same concurrency the sweep will use.
- [x] **D.2** Re-run any watchdog trip **serially, alone**, before scoring it as a hang.
- [x] **D.3** Record both outcomes — tripped-under-load and confirmed-serially — so a unit whose
      mutants are creeping toward the limit is visible before it starts producing false kills.
- [x] **D.4** **Red test:** a genuinely non-terminating mutant must still be killed. Prove the
      re-run does not launder real hangs into survivors — that error runs the other way and
      deflates the score.

**Notes:**

---

## 10. Phase E — prove it still measures the same thing

The gate's Phase C is the model: it reproduced 50,605 values exactly. The equivalent here is a
sweep whose per-mutant dispositions match a known-good serial sweep, mutant for mutant.

- [x] **E.0** **THE SWEEP RUNS ON A CLEAN TREE, NOT THIS ONE.** Discovered while starting E:
      `mutation/ParseInAry_Opt.json` records `compared_against:
      fortran_reference_on_a_clean_tree`, and `vit_mutate.py` refuses outright on an integrated
      tree ("compares the mutant against itself and the number is not a measurement"). Phase P
      was therefore verified in the **post-integration** harness mode, not the mode a sweep uses.
      E must run inside a `reset_to_clean.sh` / `restore_integrated.sh` window — they are a pair.
- [x] **E.1** Re-run ParseInAry_Opt's sweep serially on the repaired harness (Phase P) to get a
      current, trustworthy reference. The existing `mutation/ParseInAry_Opt.json` was produced
      against a harness state that no longer links; it is not a safe reference.
- [x] **E.2** Re-run the same sweep at 8 workers.
- [x] **E.3** Compare **per mutant id**, not just the score. Same killed set, same survivor set,
      same `nocompile` set, same `unreachable`/`equivalent` handling. A matching 0.9922 built
      from a different set of kills is not a match.
- [x] **E.4** Confirm the single known survivor `07b5ee72` survives in both.
- [x] **E.5** Run it twice at 8 workers and confirm the two agree — concurrency bugs are often
      intermittent, and one green run does not distinguish "correct" from "got lucky".

**Notes:**

Done 2026-08-19 inside the reset window. **All 153 mutants, serial vs 8 workers, per mutant id:**

```
mutants A=153 B=153  only-in-A=0  only-in-B=0  differing=0
per-mutant dispositions identical: True      score 0.8421 both
```

Run twice at 8 workers; the two parallel runs also agree with each other exactly. The 24-mutant
subset agreed first, also twice. `identity_verified` reads 153 on each parallel run and 0 on the
serial one — correct, because one workspace has nothing to prove.

**14.9x wall clock, and ONLY ~8x OF IT IS PARALLELISM.** From `.vit/cycle_log.jsonl`:

```
serial (--workers 1)    2,075.7 s     34.6 min
parallel (--workers 8)    139.0 s      2.3 min      14.9x
```

That is more than eight workers can produce, so the surplus needed an explanation rather than a
celebration. Paired control on the same 16 mutants:

| 16 mutants, `--sanitize` | wall |
|---|---|
| `--workers 1` (harness dir, a macOS **bind mount**) | 287.6 s |
| `--workers 2` (container-local `/tmp`) | 75.6 s — **3.8x** |

**Two workers cannot give 3.8x.** So ~1.9x of the full 14.9x is not parallelism at all: it is
moving the build off the bind mount into container-local `/tmp`, which the worker workspaces do
as a side effect. The remaining ~7.8x of a possible 8 is the pool, ~97% efficiency — higher than
the 87% in §0 because the clean tree's cycle is longer and fixed overhead weighs less.

**That ~1.9x is available to the SERIAL path too, and is not claimed by this change.** A
`--workers 1` sweep that built in `/tmp` should finish in ~18 min rather than ~35, on any tree,
with no concurrency and therefore none of §4's risk. Recorded as follow-up F.7 rather than done,
because it is a separate change with a separate proof obligation.

Reset window closed with `restore_integrated.sh`; the tree gated **PASS, 5,252,000 compared,
0 mismatched** afterwards, and the translation hashed unchanged (`5e7d1b8b`) after both serial
sweeps — the in-place mutate/restore held.

---

## 11. Phase F — turn it on

- [x] **F.1** Change the default to 8 only after E passes twice.
- [x] **F.2** Tests in `translation-loop/tests/`, and every new file named in `PLAN.md` §2.6.
- [x] **F.3** Full suite green (528 passed / 2 skipped at `6ec37f9` is the current bar).
- [x] **F.4** Update `RUNBOOK.md` with the serial fallback and when to reach for it.
- [ ] **F.5** Publish both instrument repos and verify by `git ls-remote`, per the unit-close
      protocol.
- [ ] **F.7** **Build the serial path in `/tmp` too.** Phase E measured ~1.9x sitting in the
      filesystem, independent of concurrency, and `--workers 1` does not get it today.
- [ ] **F.6** Watch the first real dispatch: `mutant_workers: 8` in the artifact, identity
      verified for every mutant, zero watchdog trips, sweep wall clock near 11 min.

**Notes:**

Done 2026-08-19, inside a `reset_to_clean.sh` window.

**The clean tree costs 2.5x more per mutant than the tree I first measured on, and parallelism
pays MORE there, not less.** A mutant cycle is 14.8 s here against the 6.0 s measured on the
post-integration harness — the clean tree runs the real Fortran reference on every case, where
the post-integration harness's "Fortran" side is a wrapper into the same C++. 14.8 s is what
this campaign's own `merged_why` estimated, so the discrepancy noted in §0 is now closed in
favour of the campaign's number. Because the extra cost is all in the RUN, and the run
parallelises, the speedup went UP:

| 24 mutants, `--sanitize` | serial | 8 workers | |
|---|---|---|---|
| clean tree (what a sweep uses) | **355 s** | **47.8 s** | **7.43x** |
| post-integration harness (§0) | 143.8 s | 20.7 s | 6.93x |

**Equivalence, per mutant id and not by score:** 24 of 24 identical — same killed set, same
survivor set (`2f8001ae`, `e6bc72cb`), zero differing, zero only-in-one. Run twice at 8 workers
(E.5); the two parallel runs also agree exactly. `identity_verified: 24` on each parallel run
and `0` on the serial one, which is correct — with one workspace there is nothing to prove.
Both artifacts carry `compared_against: fortran_reference_on_a_clean_tree`.

**Phase P's repair holds in BOTH tree states, now measured in both directions rather than
argued:**

```
clean tree       int2lstr_c/findline_c/getwords_c bridged to Fortran, objects OUT of LIBS  -> links
integrated tree  all three detected as wrappers, not bridged, objects IN LIBS              -> links
```

---

## 12. Risks, stated plainly

| risk | severity | mitigation | residual |
|---|---|---|---|
| Cross-worker source contamination → wrong score, silently | **high** | Phase C identity hash, red-tested | low |
| Watchdog false kills under load → inflated score | medium | Phase D serial re-run | low |
| Invariant-layer change inherited by all future campaigns | medium | tests, `--workers 1` preserved as the way back | low |
| Workspace build fails → silent serial fallback | medium | B.3 refuses instead of falling back | low |
| Memory exhaustion at 8-way ASan | **none measured** | 3.7 GB of 7 GB at peak | none |
| Absolute-time projection wrong (6 s vs `merged_why`'s 14 s) | low | ratio is what was measured; minutes may differ | low |

**The honest summary of the risk:** the gate was safe to parallelise because its parallel unit
was a reader. This one is a writer, and the failure mode is a score that is too high rather than
a run that is too slow. Phases C and D are not padding — without them this change is not worth
making, because an unverifiable 1.000 is worse than a slow 0.992.

**Notes:**
