# Int2LStr — unit #10

`FUNCTION Int2LStr(Num)` returning `CHARACTER(11)`. Formats a default INTEGER
right-justified into an 11-character field (`WRITE (Int2LStr,'(I11)') Num`) and
then left-justifies it (`Int2LStr = ADJUSTL(Int2LStr)`).

**The first FUNCTION RESULT in this campaign.** Every unit before it was a
SUBROUTINE, or a function whose result was a scalar returned by value. A
`CHARACTER(n>1)` result is neither: `result_is_out_param` makes it a trailing
`char*` parameter the caller owns and the callee blank-fills.

## What each file is

| file | what it records |
|---|---|
| `int2lstr.final.cpp` | the committed translation |
| `vit_interface.stdout.txt` | the shipping bridge, read before any C++ was written |
| `vit_check.stdout.txt` | 2 findings, BOTH from other procedures in the same file — see below |
| `kernel.verify_fields.csv` | the real translation, 1/1 IDENTICAL |
| `kernel.noop-stub.verify_fields.csv` | no-op — FAILS. The kernel is alive. |
| `kernel.wrong-constant-stub.verify_fields.csv` | a wrong literal — FAILS. The comparison is alive. |
| `kernel.wrong-padding-stub.verify_fields.csv` | right digits, `'X'` padding — FAILS. |
| `kernel.constant-stub-PASSES.verify_fields.csv` | **the right literal, reading no argument — PASSES.** |
| `kernel-state.Int2LStr.0.0.1` | the single captured case |
| `kernel-generated-ReadSetParameters.post-verify.f90` | the callsite file; its `!local verify variables` names `ol_string` and nothing else |
| `mutant_equivalence_probe.cpp` / `.txt` | the exhaustive proof behind the five declarations |
| `gate.control-getwords-perturbed-MOVES.json` | a known-red perturbation re-run on THIS build |

## The kernel is a lookup table, and four stubs say so rather than two

`Int2LStr` has one live call site in all 27 scenarios — `ReadSetParameters.f90:753`,
which runs **once, in scenario 24**. Coverage: every other call site is 0 in all
27 (16 of them are inside `ErrVar%ErrMsg` construction, two are in `ROSCO_IO`'s
debug-file format strings). So `vit extract` captures 1 case whatever
`vit.yaml`'s `invocation` says — unit #6's `GetPath` shape.

Unit #6 prescribed two stubs, unit #7 a third. This unit ran four, because the
kernel here compares something none of those three anticipated: **the caller's
variable, not the unit's result.** KGen instruments the assignment statement, so
the compared field is `ol_string`, and `Int2LStr`'s own output reaches it only
through `TRIM(Int2LStr(I))`.

    no-op            FAILS   — the comparison is alive
    wrong constant   FAILS   — and alive on the VALUE, not just on presence
    wrong padding    FAILS   — the padding bytes reach ol_string after all,
                               because 'X' is not a blank and TRIM keeps it
    right constant   PASSES  — reading NO argument. One input, one answer.

The fourth is the verdict: the kernel cannot distinguish this translation from a
program that ignores `Num`. `vit verify` said so itself — `NON_DISCRIMINATING`,
no by-value floating-point argument, red test not constructed — and the stubs are
the measurement it declined to make.

The padding stub is worth keeping for a reason of its own. It looks like it
proves the kernel sees all 11 bytes, and it does not: it passes through `TRIM`
only because `'X'` is non-blank. A translation that padded with a *different
blank* would be invisible here. A stub that fails does not tell you why.

## `vit check`'s two findings belong to other functions

`narrowing-local` on `ExpUCVarName` and `delimiter-set` on `':/'` are from
`CheckParseStatus` and `GetPath`, hundreds of lines away in `ROSCO_Helpers.f90`.
The translation contains no CHARACTER local and no delimiter set at all. This is
the file-scoping behaviour unit #4 recorded; re-attributed before acting on it.

## The five declared-equivalent mutants

Every one is a buffer-bound mutant on an 11-byte result:

    A  line 39  (Num <  0) -> (Num <= 0)   magnitude ternary; both arms are 0 at Num == 0
    B  line 39  (Num <  0) -> (Num <  1)   the same site, same reason
    C  line 67  first + i <  W -> <=       writes a byte the next loop overwrites
    D  line 70  i <  W -> <=               writes index 11 only
    E  line 24  const int W = 11 -> 12     every index shifts by one, off the end

`mutant_equivalence_probe.cpp` does not argue any of that. Following unit #8's
rule — *prove it exhaustively rather than reasoning about it, with the bytes past
the buffer set to the value most able to disagree* — it runs the original and all
five against a poisoned guard byte over **4,038,021 inputs**: every value with
|Num| <= 2,000,000, every value within 1000 of every decade boundary to 10^9 both
signs, and every value within 1000 of `INT_MAX` and `INT_MIN`. That is exhaustive
over the only structure the function has, since its behaviour is a function of
the decimal string and can only change where the digit count does.

**differ-IN-BOUNDS 0 for all five.**
