# GetRoot — the two mutants declared equivalent, and why

`vit_mutate.py --equivalences` takes a bare list of ids, so the REASONS have to
live somewhere a reviewer can read them. This is that file, and it is committed
beside `GetRoot.equivalences.json` for the same reason `harness/ranges.toml`
carries its measurements: a judgement stated without its basis is a judgement
nobody can check.

Both were reached only after the translation had already been changed to delete
an unobservable restatement. Declaring is the LAST move, not the first — the
campaign's rule (RUNBOOK, unit #4) is remove-the-restatement, then improve the
instrument, then declare. Three instrument gaps were closed first and they took
the score from 0.648 to 0.968; see `evidence/GetRoot/README.md`.

## `6751970e` — `if (I < len_GivenFil)` → `if (I <= len_GivenFil)`

Line 153. The reference is `IF ( I < LEN_TRIM( GivenFil ) )`, whose own comment
says *"Make sure the index I is okay"* — the guard exists to establish that a
character after the dot EXISTS before the next statement reads it.

The loop gives `I <= len_GivenFil`, so the two forms differ only at
`I == len_GivenFil`. There the mutant executes `GivenFil[(I + 1) - 1]`, which is
`GivenFil[len_GivenFil]` — **one byte past the argument**. The mutant is not a
different implementation of GetRoot; it is a program whose behaviour is
undefined, and no comparison of returned VALUES can see an out-of-bounds read.
That is unit #4's measured shape, recorded in RUNBOOK: *"each surviving mutant
there was a memory error no value comparison can see."*

It is declared rather than killed because there is no input that kills it and no
instrument in this campaign that could. Stated plainly: this is a limit of
value-comparison mutation testing, not a property of GetRoot. A sanitiser build
(`-fsanitize=address`) would kill it, and that is the instrument this would need.

## `51209c13` — `char_assign(RootName, len_RootName, "", 0)` → `..., "", 1)`

Line 178, the transcription of `RootName = ''`.

**That statement is DEAD CODE in upstream ROSCO.** Reaching it requires `I == 1`
in the branch where `I` is also the end of the string — that is,
`GivenFil` is the single character `'.'`. The procedure's own first statement is

```fortran
IF ( ( TRIM( GivenFil ) == "." ) .OR. (  TRIM( GivenFil ) == ".." ) )  THEN
   RootName = TRIM( GivenFil )
   RETURN
END IF
```

which has already returned on exactly that input. No input reaches line 178, in
the reference or in the translation, so no mutant of it can change an output.

It is transcribed anyway because P7 makes the original the oracle: the
translation mirrors what ROSCO wrote, including the branch ROSCO cannot take.
Deleting it would be a change to the reference's structure justified by an
argument, and the argument is written here instead.

This is a finding about upstream ROSCO, not about the translation, and it is the
second one this campaign has recorded in `GetRoot`'s neighbourhood — the first
being the two `Flp_Mode=2` indexing bugs in `RUNBOOK.md`.
