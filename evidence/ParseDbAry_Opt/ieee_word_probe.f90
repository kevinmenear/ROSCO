! Does gfortran's LIST-DIRECTED READ of an IEEE word give the SAME 64 BITS as
! the translation's `strtod` path?
!
! WHY THIS PROBE EXISTS. The differential harness compares `Ary` with `memcmp`
! (harness/emit.py: "memcmp, never ==: -0.0 vs +0.0 and NaN vs NaN are real
! discriminators"). Three of this unit's surviving mutants live in `match_word`
! and the only corpus record that reaches them spells `nan`, `inf` or
! `infinity` at a value position. Planting one of those makes `Ary` hold a
! non-finite value on BOTH sides -- and if the two sides spell the same NaN
! with different payload bits the corpus change turns the harness RED for a
! reason that has nothing to do with the mutants it was added for.
!
! So the bits are measured BEFORE the corpus is changed, not after it goes red.
! The reference side is here; `ieee_word_probe.cpp` runs the same words through
! the SHIPPED translation's own `parse_real`, and `run_ieee_word_probe.sh`
! prints the two columns side by side.
!
! The record layout mirrors the one the corpus will produce: one value in a
! CHARACTER(2048) internal file, blank-padded, read list-directed into a
! REAL(8) -- the reference's own `READ (Line,*,IOSTAT=ErrStatLcl) Ary`.
PROGRAM ieee_word_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxLineLength = 2048
    INTEGER, PARAMETER :: NW = 10
    CHARACTER(MaxLineLength) :: Line
    CHARACTER(32) :: words(NW)
    REAL(8) :: v
    INTEGER(8) :: bits
    INTEGER :: i, ios

    words(1)  = 'inf'
    words(2)  = '-inf'
    words(3)  = 'INF'
    words(4)  = 'Infinity'
    words(5)  = '-Infinity'
    words(6)  = 'nan'
    words(7)  = 'nAn'
    words(8)  = 'NAN'
    words(9)  = '-nan'
    words(10) = 'NaN'

    DO i = 1, NW
        Line = words(i)
        v = -987.654d0
        READ (Line, *, IOSTAT=ios) v
        bits = TRANSFER(v, bits)
        WRITE (*, '(A,A,A,I0,A,Z16.16)') 'REF ', words(i), ' iostat=', ios, &
                                         ' bits=', bits
    END DO
END PROGRAM ieee_word_probe
