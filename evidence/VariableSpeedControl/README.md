# VariableSpeedControl — unit #60, evidence index

`Controllers.f90:212-365` in the clean source (54dd134). The generator-torque
controller: choose a demanded torque by one of three control laws, apply the
shutdown ramp, saturate against the most stringent maximum, rate-limit, override
from an open-loop table, write `avrSWAP(47)`. Called from one site,
`DISCON.F90:117`, unguarded. Coverage records that CALL in 23 of the 27
scenarios and in none of 10, 13, 14 and 24 — so those four do not reach the call
statement at all, and the reason is upstream of this unit rather than in it.
(Three of the four, 10, 14 and 24, are the `OL_Mode > 0` scenarios on which
`SetParameters` RETURNs early over the absent `OL_Mode2_Input.dat`, measured at
units #17, #26 and #29; scenario 13 was not traced here.)

Six callees, all six already translated and all six CALLED rather than inlined:
`PIController` (#33, x1 or x2), `saturate` (#24, x1 or x2), `ratelimit` (#46),
`interp1d` (#23, x0..2), `unwrap` (#26), `PIDController` (#34). Three of them
post-increment an `objInst` counter, so the ORDER of the calls decides which
`piP` / `rlP` slot each one owns.

## DISPOSITION: `deferred`, and the reason is one measurement

**The differential harness could not be built, so there is no mutation score,
and P12 is what a unit closes on.** `evidence/VariableSpeedControl/corpus_wall.txt`
has the whole measurement; the two numbers are:

    143,261 cases planned      probe_corpus_count.py, 92 s
    188 KB per case            probe_corpus_size.py, a FLAT slope over 14 points
    ------------------------
    27.0 GB                    against a 7.736 GiB VM; ~41,000 cases fit

The cause is one line of the REFERENCE, and it is a real read:

    Controllers.f90:346
      LocalVar%GenTqAz = PIDController(..., LocalVar%piP, LocalVar%restart,
                                       objInst, LocalVar)

`harness/statevary.py::read_set` marks an argument read WHOLE when the body
hands it to a callee bare, so `LocalVar`'s 179 C parameters are all varied —
**224 varied parameters against IPC's 55**, the campaign's previous largest, and
FlapControl's 21 one procedure away. R6's three biggest sub-counts iterate the
DEFAULTED scalar reals, of which there are 160 here.

This is not "the VM is small". At 143,261 cases the emitter would write ~5.2 GB
(and has a measured SIGKILL ceiling between 84,754 and 95,310 cases in this same
VM), and `vit_mutate.py` re-runs the whole corpus per mutant.

`integrated_unexercised` was considered and rejected: this campaign reserves that
word for units NO SCENARIO REACHES, and 23 of 27 scenarios call this one 407,976
times.

## The layers

| layer | result | red-tested |
|---|---|---|
| translation (`translations/Controllers/variablespeedcontrol.cpp`) | transcribed statement for statement; `vit check` 15 checks, no known-shape defects | — |
| integration (`rosco/controller/src/Controllers.f90`) | `--reverse-copy`; the wrapper was READ before it was believed (unit #51's rule): three copy-backs, the `localvariables` one carries this unit's thirteen scalar writes; `avrSWAP` crosses directly as `REAL(4)` assumed-size | the whole gate below is taken against this build |
| gate, 27 scenarios (`gate/VariableSpeedControl.json`) | **5,252,000 values / 351 channels, 0 mismatched** | **three, one of them a NEGATIVE CONTROL** — the return path **1,552,676** across EXACTLY the predicted 23 scenarios, the K\*Omega² Region-2 arm **49,659** across EXACTLY `{12}`, and a 1.0e30 write on the Region 3 constant-torque arm (0 hits in all 27) **0**, predicted exactly. All three revert-verified at 0 |
| kernel replay, scenario 12 (`kernel.verify_fields.csv`) | **60/60 passed, 26,040 field rows over 419 field names, every row IDENTICAL** | `vit verify` printed NON_DISCRIMINATING, so **three hand stubs, ALL THREE PREDICTIONS EXACT**: the no-op 0/60, the `avrSWAP(47)` write deleted **60/60 PASS** (the KGen assumed-size blindness, measured), the Region-2 arm doubled **12/60** — the 12 being invocations 3..14, the `vs_state == 1` cases |
| differential harness | **NOT TAKEN** — `corpus_wall.txt` | — |
| mutation score | **NOT TAKEN** — needs the corpus | — |
| post-integration harness | **NOT TAKEN** — needs the corpus | — |

## Files

| file | what it is |
|---|---|
| `corpus_wall.txt` | the two probe results, the arithmetic, and three levers priced and refused with the reason |
| `probe_corpus_count.py` | 143,261 — collapses each array value AFTER the fill, so the rng stream and therefore the COUNT are a real run's and the bytes are not |
| `probe_corpus_size.py` | 188 KB/case, run under `ulimit -v` so the failure is a `MemoryError` with a traceback rather than a SIGKILL with nothing |
| `inputs_census.py` | committed NOT having run, saying so — it keeps the full cases so it dies where the harness died. It is the FIRST thing to run when a corpus for this unit exists (unit #59's rule for a unit whose statements divide) |
| `gate.redtest_predictions.txt` | the three perturbations, predicted from the coverage file BEFORE the runs, with the results appended after |
| `kernel.stubs.txt` | the kernel layer: why scenario 12, the arm partition read out of the field log, the three stubs with every EXPECT committed before its RESULT, and what the layer does not cover |
| `kernel.verify_fields.csv` | the 26,040 rows, 419 field names, all IDENTICAL — and the file that says `avrswap`, `piP` and `rlP` are not among them |
| `variablespeedcontrol.kstub-{noop,no-avrswap,region2}.cpp` | the three stubs, each carrying its own prediction in its header |
| `run_one_kernel_stub.sh` | copied from `evidence/YawRateControl/run_one_kernel_stub.sh` (P4), three names changed |

## Three things worth carrying

**A UNIT'S CORPUS SIZE IS DECIDED BY WHICH ARGUMENTS IT HANDS TO A CALLEE, NOT
BY HOW BIG ITS BODY IS.** This body is 100 lines and reads 22 `CntrPar` fields.
It is the single bare `LocalVar` in the `PIDController` call — inside a block
that is DEAD in all 27 scenarios — that puts 179 parameters into the varied set
and multiplies the corpus fourfold over the campaign's largest. `--dump-plan`
prints `N parameter(s) varied` in **zero seconds and before any generation**, and
that number is the price of the harness. Read it first for any unit that passes a
whole derived type to a callee.

**AN OOM-KILLED GENERATOR REPORTS NOTHING, AND `ulimit -v` IS WHAT TURNS THAT
INTO A MEASUREMENT.** The first failure was `exit 137 after 68s` and nothing
else — no count, no traceback, no artifact. Under an address-space cap the same
run raises `MemoryError` at `generate.py:2150` with the case count in hand. Two
probes and about two minutes turned "it does not work" into two numbers and an
arithmetic.

**A NEGATIVE CONTROL IS WORTH MORE WHEN THE UNIT'S OTHER INSTRUMENT IS MISSING.**
With no differential harness, the question "is the gate seeing the unit or seeing
the edit" carries most of this unit's credibility. RT2 answers it by naming the
scenario set `{12}` before the run and measuring exactly `{12}`; RT3 answers it
by moving 0 with a 1.0e30 write one statement away. Ten arms of this unit are
dead in all 27 scenarios and are listed by line in
`gate.redtest_predictions.txt` — that list is the named gap, and it is exactly
the region the missing corpus was for.

**WHEN ONE LAYER IS MISSING, THE ONE THAT NEEDS NO CORPUS IS STILL AVAILABLE —
AND ITS BLIND SPOT IS THE OTHER LAYER'S STRENGTH.** The KGen kernel replays
captured runtime state, so it needed nothing the generator could not build, and
it compares 419 FIELDS where the gate compares 13 simulation channels: `genartq`,
`genbrtq`, `gentq_sd`, `vs_maxtq`, `vs_komega2_gentq`, `vs_constpwr_gentq`,
`vs_lastgenpwr`, `instpi` and `instrl` are outputs no gate channel reads. It is
also blind to `avrSWAP(47)` — the unit's whole output — and that is MEASURED
here rather than inherited: deleting the write leaves 60/60 PASS, while the gate
red test on the same statement moves 1,552,676. The pair covers what neither
does alone. What neither covers is the contents of the nested `piP`/`rlP` blocks,
and that is left open and named rather than argued away.
