# The sanitiser amendment, demonstrated on the mutant it was proposed for

**This changes no default, writes no score, and `mutation/FindLine.json` does
not know it happened.** It is evidence for a decision the Driver has to make,
not a re-scoring of this unit.

`ca75abea` — `const_tweak '2048' -> '2049'` on `MaxLineLength` — is the one
mutant of this unit that no value oracle can kill. It reports **0 of 2514**
against the differential harness and **0 of 5,252,000** against the gate, and
both zeros have a positive control at the same site (60 of 2514, and 1,583,216
of 4,732,000), so it is genuinely invisible rather than sitting behind a corpus
that is too narrow.

`DECISIONS.md` has proposed a sanitiser build as the instrument for this whole
class since unit #31, across three units, **on an argument**. Two questions that
argument does not answer:

1. does the instrument fire on the mutant at all, on this toolchain?
2. does it stay **silent on the correct program**?

The second decides whether the amendment is one CMake option or a project. A
sanitiser that reports on the unmutated translation would mark every mutant
killed and make the score meaningless — a green that established nothing, one
sign flipped.

## What it reports

```
original          ./test exit 0,     0 byte(s) on stderr
mutant-ca75abea   ./test exit 1,  4475 byte(s) on stderr
```

```
ERROR: AddressSanitizer: heap-buffer-overflow ...
WRITE of size 1 at 0x51d0000ac680 thread T0
    #0  char_assign            findline.hpp:74
    #1  FindLine(...)          findline.hpp:219
    #2  main                   findline_test.cpp:116

0x51d0000ac680 is located 0 bytes after 2048-byte region [0x51d0000abe80,0x51d0000ac680)
allocated by thread T0 here:
    #8  main                   findline_test.cpp:106      <- std::vector<char> Line_a(len_Line_a)
```

The report names the write, the line, the buffer and the allocation. **Zero
bytes after a 2048-byte region** is the mutant stated exactly.

`original.asan.txt` is committed **empty**, and that is the artifact rather than
an absence: the correct program produces no diagnostic at all. Leak detection is
off (`ASAN_OPTIONS=detect_leaks=0`) because libgfortran's I/O buffers are
reachable at exit and are not what this measures — with it on, the correct
program reports too, and question 2 gets a false yes.

## What it does NOT answer

Whether the **other 31 translations** are clean under it. That is what the
amendment actually turns on — it changes what "killed" means for every unit at
once — and it is a sweep, not a probe. This says only that the instrument works,
that it is silent on one correct translation, and that it kills the mutant three
units have now been unable to reach.

Nor does it make `ca75abea` a kill. The score in `mutation/FindLine.json` is
`24/25 = 0.960` and stays there, because the mutation instrument for this
campaign is `vit_mutate.py` over the differential harness, and changing that
instrument for one unit is the thing X3 forbids.

## The capture bug this script had first, kept because it is the interesting one

The sanitiser's stderr was first redirected to a path **relative to the
container's working directory**, which the script had just changed to the
harness directory. Nothing was written, and both runs reported `exit 1` — the
correct program and the mutant failing identically, which is precisely what a
sanitiser firing on the Fortran runtime would look like. A capture that silently
lands somewhere else does not fail to answer the question; it answers it with
the wrong sign.

## Reproduce

```bash
bash scripts/reset_to_clean.sh
bash scripts/harness.sh FindLine ROSCO_Helpers findline \
     rosco/controller/src/ROSCO_Helpers.f90 --against translation
bash evidence/FindLine/run_asan_demo.sh
bash scripts/restore_integrated.sh
```

`run_asan_demo.sh` builds with `CXX='g++ -fsanitize=address -g
-fno-omit-frame-pointer'`, which covers both the compile and the link because
the generated Makefile spells both with `$(CXX)`. The campaign's Fortran objects
are **not** instrumented and do not need to be: the buffer that overflows is
`std::vector<char> Line_a`, allocated by the instrumented test.
