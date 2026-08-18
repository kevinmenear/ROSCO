# The fourteen declared equivalences for unit #53 `IPC`, and why

`mutation/IPC.equivalences.json` is a bare list of ids -- that is the format
`vit_mutate.py --equivalences` takes, and it carries no reasons. This file is
the reasons. **Every one of them is meant to be read and disputed**: a declared
equivalence leaves the denominator, so a wrong one raises the score by hiding a
defect, which is the one failure mode a mutation score cannot report on itself.

The claim being made in each case is the same claim: **the mutated program and
the original agree on every ADMISSIBLE input**, not merely on every input this
corpus contains. Where the argument is "the corpus does not contain such a
case", it is NOT here -- those are in `README.md` under the eight survivors, as
(b) or (c).

Line numbers are `translations/Controllers/ipc.cpp` at `811d6842`.

---

## 1. The four fixed-size procedure LOCALS, eight mutants

    L170  double PitComIPC[3];        293e1cfa  index_offset '[3]' -> '[3 + 1]'
    L171  double PitComIPCF[3];       a4268819                aed80394  const_tweak '3' -> '4'
    L172  double PitComIPC_1P[3];     1ed626ad                e4ee88fc
    L173  double PitComIPC_2P[3];     a142c881                3dcdfa38
                                                              305b22ce

Both operators make the same edit -- an array of extent 3 becomes an array of
extent 4 -- and the argument is one argument in three parts.

**The extra element is never WRITTEN.** `PitComIPC` and `PitComIPCF` are written
only at `PitComIPC[K - 1]` and `PitComIPCF[K - 1]` inside `DO K = 1,NumBl`, so
the highest index written is `NumBl - 1`. `PitComIPC_1P` and `PitComIPC_2P` are
the output arguments of the two `ColemanTransformInverse` calls, and that callee
writes exactly three elements on every invocation (`Controllers.f90`, its whole
body is three assignments).

**The extra element is never READ.** Same three subscripts, same bound.

**`NumBl <= 3` IS THE ADMISSIBLE DOMAIN AND NOT A PROPERTY OF THIS CORPUS.** The
reference's own declaration is `REAL(DbKi) :: PitComIPC(3), ...` and the loop is
`DO K = 1,LocalVar%NumBl` with nothing testing `NumBl` against 3, so at `NumBl`
above 3 the REFERENCE writes past its own locals and past `IPC_PitComF(3)`.
That is why `harness/ranges.toml` states the bound, and the entry there gives
that reason. An input on which the reference has no defined behaviour is not an
input the two programs can be compared on.

**And nothing compares the four arrays.** They are procedure locals, not fields:
they do not appear in `R4_compare_all_outputs`' 480 out-parameters. So even the
extent itself is not observable -- only what is written THROUGH the array is,
and that is `LocalVar%IPC_PitComF(K)`, which the mutation does not touch.

*How to dispute this:* show an admissible input with `NumBl > 3`, or a read of
any of the four arrays at an index outside `[0, NumBl)`.

---

## 2. `LocalVar%restart` crossing the boundary as an int, three mutants

    L249  LocalVar->restart ? 1 : 0, &objInst->instSecLPF,      f4319f87  '1' -> '2'
    L276  &LocalVar->piP, LocalVar->restart ? 1 : 0,            9ea2b033
    L436  &LocalVar->piP, LocalVar->restart ? 1 : 0, ...);      479d5d80

The mutant passes 2 where the translation passes 1. **Both consumers convert
with a comparison against zero, and both were READ rather than assumed.**

The harness's reference side reaches the callee through the generated bridge,
which is the code the mutation sweep actually ran
(`translations/Controllers/ipc_test/ipc_callees.f90`, regenerated on the clean
tree):

    INTEGER(C_INT), VALUE :: reset
    bridge_result = REAL(LPFilter(..., iStatus, (reset /= 0), inst), C_DOUBLE)
    bridge_result = REAL(PIController(..., I0, piP_f, (reset /= 0), inst), C_DOUBLE)

and the integrated tree reaches the C++ callee directly:

    translations/Controllers/picontroller.cpp:30   if (reset != 0) {

`(reset /= 0)` and `reset != 0` map every non-zero int to the same `.TRUE.`, so
1 and 2 are the same argument on both trees. This is the same fact the
translation's own comment at L243-246 states; the difference is that the comment
asserts it and the two lines above are the code.

*How to dispute this:* find a consumer of one of these three arguments that
compares it against 1 rather than against 0.

---

## 3. The VALUE half of an optional-argument pair that is not forwarded, one mutant

    L250  0, 0.0);        622f752b  const_tweak '0.0' -> '1.0'

The pair is VIT's OPTIONAL convention: `has_InitialValue` then `InitialValue`.
The literal beside the mutated one is `0`, and the bridge branches on it:

    IF (has_InitialValue /= 0) THEN
      bridge_result = REAL(LPFilter(..., inst, InitialValue), C_DOUBLE)
    ELSE
      bridge_result = REAL(LPFilter(..., inst), C_DOUBLE)

so on this call the value is passed INTO the bridge and never forwarded out of
it; `LPFilter` is invoked without the argument and `PRESENT(InitialValue)` is
`.FALSE.`. The translated callee agrees
(`translations/Filters/lpfilter.cpp:35`, `if (has_InitialValue) ...`). The
mutant changes a value nothing reads.

*How to dispute this:* show a path on which `has_InitialValue` at this call site
is non-zero. It is the literal `0` in the same argument list.

---

## 4. A dead store to a local the reference also writes, one mutant

    L305  Y_MErrF = 0.0;      03813f34  const_tweak '0.0' -> '1.0'

`Y_MErrF` is a C++ local. It is assigned at L247 (from `lpfilter_c`) on the
`Y_ControlMode == 2` arm and read at L273 on that same arm. L305 is the ELSE
arm's assignment, and after the `IF` closes at L308 the only mention of the
name is `(void)Y_MErrF;`. So no read is reachable from L305, and the local is
not a field: it is compared by nothing.

The statement is transcribed because the reference writes it and the contract is
a statement-for-statement mirror -- which is exactly why a mutant of it is
unobservable BY CONSTRUCTION, and the translation says so at its declaration
(L182-186), before these mutants existed.

*How to dispute this:* find a read of `Y_MErrF` that L305 reaches.

---

## 5. A ternary whose two spellings agree at every value, one mutant

    L132  const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
          b1dc4faa  compare_op '>' -> '>='

This one is arithmetic rather than reachability, and it holds for every `int n`:

    n  >  0        n >= 0        the two expressions
    ----------------------------------------------------
    n > 0    n            n              equal
    n = 0    0 (literal)  (size_t)0      equal -- both are 0
    n < 0    0 (literal)  0 (literal)    equal

The only value at which the predicates disagree is `n == 0`, and there the two
branches produce the same number. `find_last_not_of` is never reached with a
different view, so the returned string is identical.

**This is NOT the same as the two `0 -> 1` mutants on the same line**, which
change *which* value the branches produce and are corpus gaps, listed in
`README.md` as (b).

*How to dispute this:* exhibit an `int n` at which the two ternaries differ.
