! The FOUR records that END INSIDE A VALUE, through the reference's own READ.
!
! WHY. Five of this unit's surviving mutants are `p < len` -> `p <= len` and one
! is `match_word`'s `p + LEN(word) > len` -> `>=`. All six differ from the
! original only when a scan reaches `p == len` -- that is, when the 2048th
! character of the record is still part of a value instead of blank padding.
! Every record the corpus produces today is a short line blank-padded out to
! `CHARACTER(MaxLineLength)`, so no scan gets within two thousand characters of
! the boundary and the six are invisible.
!
! The corpus change that reaches them plants a value long enough to fill the
! record. THIS PROBE ASKS WHAT THE REFERENCE DOES WITH ONE, before the corpus is
! changed rather than after the harness goes red -- because a 2048-digit field
! is exactly the kind of input a runtime is entitled to reject, and a planted
! record the reference rejects differently from the translation is a red harness
! that says nothing about the mutants it was planted for.
!
!   tail:digits   "0"x2047 // "1"            the INTEGER part runs to the end
!   tail:frac     "1." // "0"x2045 // "5"    the FRACTION runs to the end
!   tail:inf      "0"x2044 // ",inf"         a matched IEEE word ENDS at 2048
!   tail:nan      "0"x2044 // ",nAn"         the same, with an uppercase letter
!                                            inside the word
!
! Two elements are read, because the last two records hold two values. The
! sentinel is the same -987.654 the earlier probes use, so an element the
! transfer never reached is visible as itself.
PROGRAM record_tail_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxLineLength = 2048
    INTEGER, PARAMETER :: NR = 4
    CHARACTER(MaxLineLength) :: Line, recs(NR)
    CHARACTER(16) :: names(NR)
    REAL(8) :: v(2)
    INTEGER(8) :: b1, b2
    INTEGER :: i, ios

    names(1) = 'tail:digits'
    recs(1)  = REPEAT('0', 2047) // '1'
    names(2) = 'tail:frac'
    recs(2)  = '1.' // REPEAT('0', 2045) // '5'
    names(3) = 'tail:inf'
    recs(3)  = REPEAT('0', 2044) // ',inf'
    names(4) = 'tail:nan'
    recs(4)  = REPEAT('0', 2044) // ',nAn'

    DO i = 1, NR
        Line = recs(i)
        v = -987.654d0
        READ (Line, *, IOSTAT=ios) v
        b1 = TRANSFER(v(1), b1)
        b2 = TRANSFER(v(2), b2)
        WRITE (*, '(A,A,A,I0,A,Z16.16,A,Z16.16)') 'REF ', names(i), &
              ' iostat=', ios, ' b1=', b1, ' b2=', b2
    END DO
END PROGRAM record_tail_probe
