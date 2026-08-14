# PIDController — the one mutant declared equivalent, and why

`vit_mutate.py --equivalences` takes a bare list of ids, so the REASON has to
live somewhere a reviewer can read it. This is that file, committed beside
`PIDController.equivalences.json` for the same reason `harness/ranges.toml`
carries its measurements: a judgement stated without its basis is a judgement
nobody can check.

**DECLARING WAS THE LAST MOVE, NOT THE FIRST.** The campaign's rule (RUNBOOK,
unit #4) is remove-the-restatement, then improve the instrument, then declare.
This unit's first dispatch scored **0.759** with SEVEN survivors and declared
NONE of them — deliberately, so that the undeclared number was on the record
first (unit #26's discipline). Six of the seven were not equivalent at all: they
were unreachable, and the instrument was what had to change. Three stated
admissible ranges later they are dead, at **28 of 29 = 0.966** with the same
declaration still absent (`mutation/PIDController.undeclared.json`, and the
before/after censuses that drove the change are in
`evidence/PIDController/clamp_census.{before,after}.csv`).

Only then this one, and only because it cannot be reached by any input.

## `15c8567c` — `lpfilter_c(..., 0, 0.0)` → `lpfilter_c(..., 0, 1.0)`

`translations/Controllers/pidcontroller.cpp:35-37`, the last argument of the
`LPFilter` call. VIT renders the Fortran OPTIONAL `InitialValue` as TWO C
arguments — a presence flag and a value — and this call site passes no
`InitialValue`, so the flag is the literal `0` immediately before it.

**THE VALUE IS NEVER READ WHEN THE FLAG IS ZERO, ON BOTH SIDES OF EVERY
COMPARISON THIS CAMPAIGN MAKES.**

The harness's reference side reaches the Fortran `LPFilter` through the
generated bridge, which does not pass the argument at all:

```fortran
IF (has_InitialValue /= 0) THEN
bridge_result = REAL(LPFilter(..., inst, InitialValue), C_DOUBLE)
ELSE
bridge_result = REAL(LPFilter(..., inst), C_DOUBLE)          ! <- this arm
END IF
```
`translations/Controllers/pidcontroller_test/pidcontroller_callees.f90:40-44`.
In the taken arm `InitialValue` is not an actual argument of anything;
`PRESENT(InitialValue)` inside `LPFilter` is `.FALSE.` and `InitialValue_` keeps
its default of `InputSignal` (`Filters.f90:49-50`).

The integrated build reaches the shipped C++ `LPFilter`, which gates the read
with the same flag:

```cpp
double InitialValue_ = InputSignal;
if (has_InitialValue) InitialValue_ = InitialValue;
```
`translations/Filters/lpfilter.cpp:34-35`.

So the mutated program and the original agree on every admissible input, not
merely on the 9758 this corpus contains. The difference is a value written into
a dead parameter.

**THE GATE ARGUMENT ITSELF IS NOT EQUIVALENT AND IS NOT DECLARED.** The mutant
one position to the left — `const_tweak '0' -> '1'` on `has_InitialValue`, id
`68137aa4`, the same line 37 — is **killed in 1145 of 9758 cases**: with the flag set, `LPFilter`
initialises its state from `0.0` instead of from `InputSignal`, and that reaches
`LocalVar%FP` and the return value. That pairing is what makes this declaration
checkable rather than convenient: the corpus DOES distinguish the flag, and it
is only the value the flag makes unreadable that it cannot distinguish.

**WHICH `0.0` THIS IS, ESTABLISHED TWICE.** The file holds two `0.0` literals
and the committed artifact names only `'0.0' -> '1.0'`.

  1. `harness.cppmutate.mutants()` carries a LINE with each mutant, and re-run
     over this exact translation it puts `15c8567c` at **line 37** — the
     `lpfilter_c` argument — and the other `'0.0' -> '1.0'`, `8a0f01f4`, at
     **line 78**, `piP->ELast[i] = 0.0` in the reset arm
     (`evidence/PIDController/mutant_sites.txt`).
  2. Independently, by count: `8a0f01f4` is **killed in 4874 of 9758 cases**,
     which is exactly the number of RESET-arm cases the census reports
     (`evidence/PIDController/clamp_census.after.csv`). A mutant that fails
     precisely the arm it sits in is that arm's mutant; the survivor is the
     other site.

Unit #34's rule read forward: identify by building or by counting, not by
reasoning about which line the tool probably meant.
