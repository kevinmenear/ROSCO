# GetWords — declared equivalent mutants

One, and it is declared on a MEASUREMENT rather than on an argument.

## `91681005` — `arith_op: 'len_Line - Ch' -> 'len_Line + Ch'`

    const int NextWhite = scan_first(Line + Ch, len_Line - Ch,
                                     Whitespace, (int)sizeof Whitespace);

`len_Line - Ch` is the length of the Fortran substring `Line(Ch+1:)` — what is
left of the line. The mutant asks `scan_first` to search `len_Line + Ch` bytes
from the same start, which is the correct region plus `2*Ch` bytes lying
entirely PAST THE END of the caller's buffer.

**Why it cannot be killed rather than why it was not.** The mutant's region is a
SUPERSET of the correct one with the same first byte, and `scan_first` returns
the FIRST match. So whenever the correct scan finds a separator, the mutant
finds that same separator at that same position. The two can only disagree when
the correct scan finds none — and then the mutant's answer names a byte outside
the buffer. Every execution that distinguishes them has already read memory the
program does not own.

**Measured exhaustively, not argued.** `mutant_91681005_probe.cpp` enumerates
every `Line` over the alphabet {blank, non-separator, separator} at every length
1..7 — 3,279 strings — and every reachable `Ch` for each, with the bytes after
the buffer set to SEPARATORS so the out-of-bounds region is maximally able to
disagree:

    same 19695   differ-only-past-the-end 4908   differ-IN-BOUNDS 0

`mutant_91681005_probe.txt` is that run. Zero of the 4,908 disagreements are
reachable without reading past the end.

**Why the site is not deleted instead.** That is what unit #7 required of an
out-of-bounds-only survivor, and it is what was done here for the four that
`all_blank` used to admit — `std::all_of` removed the index expression and all
four went away. This site is different: it is the LENGTH OF A SUBSTRING, and the
boundary VIT generates is `(char* Line, int len_Line)`. Fortran carries a
substring's length implicitly; C cannot, so with this calling convention the
subtraction is irreducible. Restructuring `scan_first` to take a begin/end
pointer pair would move the site rather than remove it — `Line + len_Line` would
then be the mutable expression, and its mutant would die only by running away
off the end, which is the same undefined behaviour scored as a crash instead of
as a survivor. Declaring it is the honest bookkeeping; disguising it as a crash
kill is not.
