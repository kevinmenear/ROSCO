# The mutation run measured the mutant against itself

```
mutants 173   killed 4   survived 169   score 0.0231
nocompile 19 of 192 (9.9%, under the 25% refusal limit)
```

**169 survivors on a corpus of 16,769 cases that passes 0-failed against real
Fortran is not a fact about the translation.** A single `compare_op` flip in a
predicate the corpus reaches must move an output; 40 of them cannot all be
equivalent.

## What actually happened

`vit_mutate` runs `make -C <unit>_test test`, and on an INTEGRATED tree that
build has both sides of the comparison ending in the same code:

```
checkinputs_f90  (the harness's Fortran side)
  -> CheckInputs           the wrapper `vit integrate` wrote
  -> checkinputs_c         -> vit_integration_shim.o
  -> CheckInputs(...)      the harness's OWN compiled copy -- the MUTANT
```

So the mutant is compared against the mutant. Every behavioural difference
cancels, and the 4 kills are the `nocompile` and crash cases.

The shim is in that link because `harness.sh` drops `<stem>.cpp.o` and the
integrated wrapper then has an undefined `checkinputs_c`. Supplying it makes
the build succeed and makes the measurement vacuous — which is the shape unit
#21 and unit #24 both warn about: *a green that measured nothing*. Here it is a
RED that measured nothing, which is the same failure with the sign flipped.

## What the measurement requires

The reference side has to be the ORIGINAL Fortran, so the mutation run belongs
on a CLEAN tree — the same configuration the pre-integration harness ran in,
where `ReadSetParameters.f90.o` carries the real 857-line body and nothing
routes back into the translation. `harness/CheckInputs.json` (16,769 / 0
failed) was taken there and is valid; this run was not.

**NOT RE-RUN in this dispatch, and the reason is stated rather than hidden:**
it needs `reset_to_clean.sh`, a full rebuild, a harness regeneration and a
~50-minute mutation sweep, and the session did not have that budget left after
six tool defects. The artifact is committed as
`mutation.integrated-build-INVALID.json` so the number cannot be read as this
unit's score, and `mutation/CheckInputs.json` carries the same run — P12 fails
it either way, at 0.0231 against a threshold of 1.0.

## The rule this earns

**A mutation run must be able to say what the mutant was compared AGAINST.**
Nothing in `mutation/<U>.json` records whether the reference side was Fortran or
the translation itself, and the two produce scores at opposite ends of the range
from the same mutants. `harness/<U>.postintegration.json` carries a `measures:`
field saying exactly this for the harness; the mutation artifact has no
equivalent and needs one.
