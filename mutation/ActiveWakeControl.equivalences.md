# ActiveWakeControl — the ten mutants declared equivalent, and why

`vit_mutate.py --equivalences` takes a bare list of ids, so the REASONS live
here, committed beside `ActiveWakeControl.equivalences.json`. Same standing as
`GetRoot.equivalences.md`: a judgement stated without its basis is a judgement
nobody can check.

**Declaring is the LAST move.** The first dispatch of this unit declared NOTHING
and left P12 failing at 0.7857 with all 45 survivors named and grouped in
`evidence/ActiveWakeControl/mutation.census.txt`. This dispatch first repaired
the corpus — two entries in `harness/ranges.toml`, `CntrPar_AWC_Mode` written as
the enumeration it is and `CntrPar_AWC_phaseoffset` held to one turn — and only
then declared the ten below. Every one of the ten is a mutant **no input can
kill**, and each entry says which fact makes that true.

Nine of the ten are declared against the ADMISSIBLE DOMAIN rather than against
the whole of C++'s: `harness/ranges.toml` states that domain, with the
measurement forcing each bound, and this file leans on exactly two of those
bounds — `CntrPar_AWC_NumModes` in [1, 2] and `LocalVar_NumBl` in [0, 3]. Both
are there because the REFERENCE writes outside its own frame past them, which is
stated at each entry.

---

## The eight extent mutants — an enlarged array whose extra element is never named

| id | line | mutation |
|---|---|---|
| `acf8643f` | 113 | `double AWC_angle[3]` → `[4]` |
| `b2eb03aa` | 113 | `double AWC_angle[3]` → `[3 + 1]` |
| `3b46e0e1` | 134 | `static double AWC_TiltYaw[2]` → `[3]` |
| `4353c022` | 134 | `static double AWC_TiltYaw[2]` → `[2 + 1]` |
| `cde1bef2` | 135 | `static double Error[2]` → `[3]` |
| `87d2ec58` | 135 | `static double Error[2]` → `[2 + 1]` |
| `aae1839e` | 138 | `double FixedFrameM[2]` → `[3]` |
| `7dd59c0c` | 138 | `double FixedFrameM[2]` → `[2 + 1]` |

Each is a DECLARATION, not a statement: it changes the extent of an array and
nothing else. The mutant differs from the original only if some lvalue in the
function names the added element, so the measurement is a COMPLETE ENUMERATION
of every subscript on those four arrays — every one, not a sample, taken by a
regex over the translation and reproduced in
`evidence/ActiveWakeControl/mutation.extent_equivalence.txt`.

    AWC_angle    [0] [1] [2]  and  [K - 1]      K = 1 .. LocalVar->NumBl
    AWC_TiltYaw  [0] [1]      and  [Imode - 1]  Imode = 1 .. CntrPar->AWC_NumModes
    Error        [0]          and  [Imode - 1]  Imode = 1 .. CntrPar->AWC_NumModes
    FixedFrameM  [0] [1]      and  [Imode - 1]  Imode = 1 .. CntrPar->AWC_NumModes

`LocalVar_NumBl` is admissible on [0, 3] and `CntrPar_AWC_NumModes` on [1, 2], so
the largest index any of the four can take is 2, 1, 1 and 1 — in every case
strictly inside the ORIGINAL extent. The three bare uses of `AWC_angle` are the
pointer passed to `colemantransforminverse_c`, whose Fortran dummy is
`PitComIPC(3)` and which therefore writes exactly three elements whatever the
caller's array is; `FixedFrameM`'s only escapes are `&FixedFrameM[0]` and
`&FixedFrameM[1]` into `colemantransform_c`'s two scalar outputs. No load and no
store moves, and a `static` array's unlisted initialiser is zero either way.

**AND THIS IS WHY THE DOMAIN MATTERS RATHER THAN BEING A CONVENIENCE.** At
`AWC_NumModes == 3` the loop at :195 would write `AWC_TiltYaw[2]` — outside the
original array and INSIDE the mutant's. There the two programs genuinely differ.
That value is not admissible, and it is not admissible for a reason recorded
before this dispatch: the REFERENCE's own `AWC_TiltYaw` is `DIMENSION(2)` and
`AWC_NumModes = 3` writes past it in Fortran exactly as in C++. The declaration
is "equivalent on every admissible input", which is what criterion (a) asks, and
not "equivalent in C++".

### The measurement that did NOT settle it, recorded because it was taken

`evidence/ActiveWakeControl/mutation.census.txt` (G3) said these eight were
equivalent by C++ semantics but left them undeclared, naming the missing
measurement: *"a diff of the generated assembly, which was not taken."* It was
taken here, at `-O2 -fPIC -ffp-contract=off`, and **it does not answer the
question**:

    acf8643f  448 changed asm line(s)   instruction multiset identical: NO
    b2eb03aa  448                                                       NO
    3b46e0e1  155                                                       NO
    4353c022  155                                                       NO
    87d2ec58    6                                                       yes
    cde1bef2    6                                                       yes
    7dd59c0c  158                                                       yes
    aae1839e  158                                                       yes

Growing a stack array by eight bytes moves the frame size and reshuffles the
register allocator across the whole function; two of the eight even change the
instruction mix. An assembly diff can CONFIRM equivalence when it is empty and
can say nothing when it is not, and here it is not. The enumeration above is the
measurement that decides, and the census's proposal is retracted rather than
quietly dropped.

---

## `df13f212` — `AWC_TiltYaw[0] = 0.0` → `AWC_TiltYaw[1] = 0.0`, line 192

The transcription of the reference's `AWC_TiltYaw = [0.0, 0.0]` at the top of the
Mode 2 arm is two statements; the mutant makes the first of them write element 1
instead of element 0, so element 0 is not reset.

**The reset is a dead store on the whole admissible domain.** Immediately below
it, unguarded:

```
for (Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
    AWC_TiltYaw[Imode - 1] = D2R * CntrPar->AWC_amp[Imode - 1] * sin(...);
    if (CntrPar->AWC_NumModes == 1) AWC_TiltYaw[1] = D2R * CntrPar->AWC_amp[0] * sin(...);
    colemantransforminverse_c(AWC_TiltYaw[0], AWC_TiltYaw[1], ...);
}
```

`AWC_NumModes` is admissible on [1, 2], so `Imode = 1` always runs and always
writes element 0, before the only read of it in this arm. At `AWC_NumModes == 1`
element 1 is written too, by the inner `IF`; at 2 by the second iteration. Both
elements are therefore rewritten before any read, on every admissible input, and
which of them the reset touched cannot be recovered — including by a later case,
since `AWC_TiltYaw` is `static` and leaves this arm in the same state either way.

The value that WOULD separate them is `AWC_NumModes == 0`: there the loop does
not run and the reset is the only writer. That value is not admissible, and its
exclusion predates this dispatch — at 0 the Mode 2 arm reads `AWC_angle`
uninitialised, in the reference as much as in the translation, and the two sides
reach that garbage down call stacks of different depth. `harness/ranges.toml`
carries the reasoning.

**Its neighbour `e9d4580d` is NOT declared, and the census was wrong about it.**
`AWC_TiltYaw[1] = 0.0` → `AWC_TiltYaw[2] = 0.0` writes one element PAST a
two-element `static` array. The census grouped it with this one as "moving either
reset to another element changes nothing". That is right about the observable
values and wrong about the program: it is an out-of-bounds store, which is
undefined behaviour and not an equivalent program, and no comparison of returned
VALUES can see it. It stays an honest survivor, in the same category unit #4
recorded for `GetRoot`: a memory error a value-comparison instrument cannot
reach, whose instrument is a sanitiser build.

---

## `5ea58638` — the second initialiser of `Error`, line 135: `{0.0, 0.0}` → `{0.0, 1.0}`

`Error` is `REAL(DbKi), DIMENSION(2) :: Error = [0.0, 0.0]` in the reference —
implicitly `SAVE`, so its initialiser is observable only where a READ of an
element precedes every write of it, in the whole run and not just in one case.

Every mention of `Error` in the translation, complete:

    :245  Error[Imode - 1] = ...                       WRITE
    :254  rescontroller_c(Error[Imode - 1], ...)       read, same iteration, after :245
    :260  picontroller_c(Error[Imode - 1], ...)        read, same iteration, after :245
    :289  DebugVar->axisYaw_2P = Error[0]              read
    :303  Error[0] = ...                               WRITE
    :313  Error[0], :339  Error[0]                     read

Element 1 is reached only through `Error[Imode - 1]` at `Imode == 2`, and the
write at :245 is the first statement of that iteration — the two reads at :254
and :260 are the same iteration's, after it. So no execution reads element 1
before writing it, and its declared initial value cannot reach any output.

Element 0 is a different matter and is NOT declared: `:289` reads it on the path
where `LocalVar->Time <= StartTime` skipped the loop entirely, which is exactly
how the mutant on ITS initialiser was killed.

---

## What is NOT declared, and why the list stops here

Thirty-five survivors of the first dispatch's forty-five are not on this list.
The two `AWC_TiltYaw` initialisers at line 134 (`537e3fe0`, `d600d733`) are
genuinely observable — Modes 3 and 4 read both elements through
`colemantransforminverse_c` whether or not `Time > StartTime` let the loop assign
them — so they are a question for the corpus, not for this file, and the
`CntrPar_AWC_Mode` ordering added to `harness/ranges.toml` is aimed at them.
Everything else is either killed by the repaired corpus or recorded as a measured
instrument gap in `evidence/ActiveWakeControl/mutation.census.txt` and
`DECISIONS.md`. **A survivor that this campaign's instruments cannot reach is
reported as a survivor.** Declaring it would convert a corpus weakness into a
score.
