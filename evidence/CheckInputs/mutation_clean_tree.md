# CheckInputs — the mutation sweep, re-taken where it means something

**Date:** 2026-08-13
**Status:** completed
**Scope:** unit #29, P12 / E4.6

Supersedes `mutation.integrated-build-INVALID.json` and
`mutation_measured_nothing.md`, which describe the run this one replaces.

## What was run

`mutation/CheckInputs.json` previously held a run taken on an INTEGRATED tree,
where the harness's Fortran side calls the wrapper `vit integrate` wrote, which
calls `checkinputs_c`, which links the harness's own compiled copy — the mutant.
Both sides were the mutant. It carried `not_evaluable: true` and
`compared_against: "THE MUTANT ITSELF -- INVALID"`, and `vit_mutate` now refuses
that configuration outright (exit 2, naming the file that still calls
`checkinputs_c`).

Re-taken on the clean tree (`scripts/reset_to_clean.sh` →
`scripts/harness.sh ... --no-generate` → sweep → `scripts/_mutation_stamp.py` →
`scripts/restore_integrated.sh`):

```
192 mutants   19 nocompile   173 behavioural   8 killed   165 survived
score 0.0462                 compared_against: fortran_reference_on_a_clean_tree
```

The reference side is asserted rather than assumed. `_mutation_stamp.py` reads
the object with `nm`:

```
ReadSetParameters.f90.o   defines __readsetparameters_MOD_checkinputs
                          references no checkinputs_c
```

### The sweep was five foreground runs, not one backgrounded one

192 mutants at ~9.5 s each is a 32-minute sweep; a dispatch's foreground command
may block for 600 s. Backgrounding it is the failure this campaign has paid for
twice — the sweep outlives the session, orphans in the container, and writes into
the tree after the driver has moved on. So it ran as five `--operator`
invocations, each blocking, and `scripts/_mutation_merge.py` unions them.

The union is a coverage claim, so it can fail. The operator population is asked
of `harness.cppmutate` — the same function `vit_mutate.py` calls — and never
derived from the parts, whose own `operators` field is computed AFTER the filter
and so lists only that part's operators. Being a per-operator COUNT it also
settles the stronger question: each part's mutant total must equal the number of
mutants its own operators produce.

```
arith_op 17  compare_op 40  const_tweak 40  drop_call 2  drop_factor 6
index_offset 33  negate_cond 40  swap_callee 2  swap_operands 12    = 192
```

Reproduce, read-only, from the five committed parts:

```
python3 scripts/_mutation_merge.py --unit CheckInputs \
    --cpp translations/ReadSetParameters/checkinputs.cpp \
    --part mutation/CheckInputs.clean.calls_arith.json \
    --part mutation/CheckInputs.clean.index_offset.json \
    --part mutation/CheckInputs.clean.compare_op.json \
    --part mutation/CheckInputs.clean.const_tweak.json \
    --part mutation/CheckInputs.clean.negate_cond.json \
    --out /tmp/remerged.json --why "reproduction"
# then diff against mutation/CheckInputs.json ignoring merged_why / the stamp
```

## THE INVALID RUN'S NUMBER WAS NEXT TO THE VALID ONE, AND THE ARGUMENT FOR RE-TAKING IT SAID OTHERWISE

This is the finding that could not be had any other way.

```
integrated tree, mutant vs itself   4 killed of 173    0.0231   INVALID
clean tree, mutant vs Fortran       8 killed of 173    0.0462   valid
```

`RUNBOOK.md`'s target-layer entry and `_mutation_stamp.py`'s own docstring both
assert that *"the same mutants scored on a CLEAN tree give a number at the other
end of the range"*. Measured, they do not: the score doubles from 0.023 to 0.046
and stays at the bottom. The reasoning that DETECTED the invalid run — "169
survivors on a corpus that passes 16,769 of 16,769 clean is the tell" — reached
the right conclusion from a premise that is false. The run was invalid; its
number was not far wrong.

So the shape-check the RUNBOOK proposes (compare `mutation/<U>.json`'s kills
against `harness/<U>.json`'s pass count) would NOT have caught this defect, and
`nm` on the reference object would have, in one command. Read the configuration,
not the number.

## WHY 165 MUTANTS SURVIVE A CORPUS THAT GOES RED ON ONE CONSTANT

Not weak coverage. The unit's outputs collapse to a single value across the
whole corpus, and both halves of that were measured on this exact build.

`CheckInputs` writes exactly two things: `ErrVar%aviFAIL` and `ErrVar%ErrMsg`
(plus `ErrStat`). There is no early return — each of ~180 checks that fails
assigns both, so **the last failing check wins**.

```
aviFAIL = -1 -> -7 everywhere         16,769 of 16,769 FAILED   (control: alive)
PROBE B  no ErrMsg ever written       16,769 of 16,769 FAILED
PROBE A  first-writer-wins            16,769 of 16,769 FAILED
the translation, unperturbed                   0 FAILED
```

Read them together:

* the control says the chain is alive and the reference side is real Fortran —
  the same run that established the 0 was capable of 16,769;
* **B** says every case in the corpus raises at least one error, so `aviFAIL` is
  `-1` in all 16,769 and can distinguish nothing;
* **A** says the FIRST failing check's message differs from the LAST one in
  every case — so every case raises **at least two** errors, and the corpus
  never contains a case where one check fires alone.

The single discriminating output is therefore the message of whichever check
happens to be last. A mutant anywhere above it changes nothing observable. That
is not a gap in the 16,769 cases; it is the shape of the corpus — the generator
varies many parameters at once, and this unit answers with one string.

`PROBE A` is one line at the top of `assign_errmsg`, which every message in the
translation goes through:

```cpp
if (ErrVar->aviFAIL != 0) return;   // PROBE A: first-writer-wins
```

Both probes must be run on the CLEAN tree with the harness relinked
(`scripts/harness.sh ... --no-generate`). On the integrated tree they measure
the mutant against itself and both report 0 — the same defect this whole re-take
exists to remove.

## What this does NOT say

It does not say the translation is wrong. Three layers still hold, and the
harness's green is a real comparison of 16,769 cases against the Fortran — it is
simply a comparison of one string per case.

It does not say the corpus is badly generated. Reaching a score here needs cases
that fail exactly ONE check, i.e. a near-valid configuration perturbed one check
at a time. No rule in `harness/` generates that, and adding one is a corpus
feature, not a repair to this unit.

It does not license declaring the survivors equivalent. `negate_cond` on
`(LoggingLevel < 0) || (LoggingLevel > 3)` is a real behavioural change that the
instrument cannot see; calling it equivalent is how a real defect gets excused.
