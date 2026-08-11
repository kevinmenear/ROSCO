# Two VIT defects found at unit #5's second dispatch, recorded before they were fixed (C12)

Both were found by trying to do the thing the first dispatch declared impossible.
Both were fixed at the source (X2), additively (P5), and red-tested.

---

## 1. A generated differential bridge longer than 132 columns

**VIT `83d25f9`.**

Decomposing a derived type emits one bridge dummy argument per field.
`ControlParameters` has 214 fields and `LocalVariables` 168, so
`vit test-validate StateMachine` produced a `SUBROUTINE` statement **11,747
characters long, on one line**.

Free-form Fortran's line limit is 132 columns. gfortran truncates past it and
then reports something else entirely, several lines later:

```
Error: Unexpected junk in formal argument list
Error: USE statement at (1) cannot follow attribute declaration statement at (2)
```

**1,153 diagnostics, not one of which names the cause.**

| | before | after |
|---|---|---|
| lines over 132 columns | 1 (at 11,747) | 0 |
| `gfortran -c` errors | 1,153 | 0 |
| lines in the file | 1,342 | 1,580 |

`statemachine_bridge.truncated_head.f90` and
`statemachine_bridge.line_lengths.txt` are the wrong artifact, kept.

**Why this matters more than one unit.** It is not that `StateMachine` failed to
build. **Every unit in this campaign taking `ControlParameters` or
`LocalVariables` — which is most of ROSCO's controllers — was outside
`vit test-validate` entirely, and nothing said so.** P11 and P12 are mandatory
for every unit and both are built on that bridge. The campaign's first four
units took scalars, arrays and strings and never touched a derived type, so this
went unmeasured until here.

The fix is in two parts and the second is the one that generalises: the dummy
list is wrapped, and then the **whole generated bridge** is passed through a rule
about line *contents*. The second exists because the first was not enough —
measured, not predicted: with the dummy list wrapped, the array copy-in
statements were still 133–153 columns, one statement holding two decomposed
field names, neither shortenable.

## 2. `CHARACTER(:), ALLOCATABLE` had no view-struct representation

**VIT `a2e2c30`.** See `../README.md` §4 for what it blocked (37 of 69 units) and
`../../../DECISIONS.md` for the design.

The remedy VIT's own error message suggested — `vit analyze-types --fix
character` — rewrites ROSCO's type definition to a fixed length, changing
`LEN(ErrVar%ErrMsg)`, which sizes `avcMSG`, which is Record 49. Changing the
oracle to make the translation checkable inverts P7. It is refused, and it is now
not the only option.

**A defect inside the fix, found by RUNNING the generated code rather than
reading it.** The populator's first draft filled its staging buffer with
`buf = src%ErrMsg`. That is a whole-variable assignment to a deferred-length
ALLOCATABLE, so Fortran 2003 automatic reallocation applies and the buffer comes
straight back out at `LEN(src%ErrMsg)` — discarding the headroom allocated one
line above and making the capacity published to C++ **a lie in the dangerous
direction**. Every shape assertion passed. The round-trip fixture reported
`cap=5` where 4101 had been allocated, and the grow case wrote 18 bytes into 4.

That is the whole argument for `tests/test_deferred_char_roundtrip.py` in the VIT
repo, which compiles and runs the generated bridge instead of inspecting its
text.
