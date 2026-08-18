! Second probe. The first (`fortran_io_probe.f90`) established the shape; this
! one closes the cases the C++ scanner has an explicit branch for, so no branch
! of the translation rests on an inference from the sibling unit's REAL probe.
!
! The question it exists to settle: unit #54's REAL parser has a ZERO-STORE
! rule -- a field with a decimal point and no digits reaches `strtod`, and the
! 0.0 it yields is TRANSFERRED before the error is raised. The first probe
! above says the INTEGER reader has no such rule (every 5010 row leaves the
! failing item at its sentinel). These records are the ones that would show it
! if it existed anywhere, plus the repeat-count and separator corners.
PROGRAM fortran_io_probe2
    IMPLICIT NONE
    INTEGER, PARAMETER :: IntKi = 4
    INTEGER, PARAMETER :: MaxLineLength = 2048
    CALL p()
CONTAINS
    SUBROUTINE one(tag, text, n)
        CHARACTER(*), INTENT(IN) :: tag, text
        INTEGER, INTENT(IN) :: n
        CHARACTER(MaxLineLength) :: Line
        INTEGER(IntKi), ALLOCATABLE :: Ary(:)
        INTEGER :: s, i
        CHARACTER(40) :: b
        Line = text
        ALLOCATE(Ary(n))
        DO i = 1, n
            Ary(i) = -987654_IntKi
        END DO
        s = 0
        READ (Line, *, IOSTAT=s) Ary
        WRITE (*, '(A,A,A,I2,A,I6,A)', ADVANCE='NO') &
            'READ  ', tag, '  n=', n, '  iostat=', s, '  ->'
        DO i = 1, n
            WRITE (b, '(I12)') Ary(i)
            WRITE (*, '(A,A)', ADVANCE='NO') ' ', TRIM(ADJUSTL(b))
        END DO
        WRITE (*, '(A)') ''
        DEALLOCATE(Ary)
    END SUBROUTINE one
    SUBROUTINE p()
        WRITE (*,'(A)') '--- separators, adjacency and the terminator ---'
        CALL one('comma-comma  ', '1,,3 Name', 3)
        CALL one('comma-trail  ', '1, Name', 2)
        CALL one('comma-eor    ', '1,', 2)
        CALL one('slash-tight  ', '1/2 Name', 3)
        CALL one('slash-eor    ', '1 /', 3)
        CALL one('semi-lead    ', ';1 2', 2)
        CALL one('semi-after   ', '1 ;2', 2)
        WRITE (*,'(A)') '--- repeat counts ---'
        CALL one('rep-signed   ', '2*-5 Name', 2)
        CALL one('rep-plus     ', '2*+5 Name', 2)
        CALL one('rep-over-n   ', '5*7 Name', 3)
        CALL one('rep-eor      ', '3*', 3)
        CALL one('rep-then-comma', '2*,9 Name', 3)
        CALL one('rep-real     ', '2*3.5 Name', 2)
        CALL one('rep-zerolead ', '02*7 Name', 2)
        CALL one('rep-huge     ', '99999999999*7 Name', 2)
        WRITE (*,'(A)') '--- would a zero-store rule show anywhere? ---'
        CALL one('dot-then-sep ', '. 5', 2)
        CALL one('dot-then-comma', '., 5', 2)
        CALL one('dot-then-slash', './ 5', 2)
        CALL one('sign-dot     ', '+. 5', 2)
        CALL one('digit-dot-sep', '5. 6', 2)
        CALL one('digit-dot-com', '5., 6', 2)
        CALL one('e-alone      ', 'e1 5', 2)
        CALL one('dot-e        ', '.e1 5', 2)
        WRITE (*,'(A)') '--- blanks, width and the record tail ---'
        CALL one('lead-blanks  ', '     42 Name', 1)
        CALL one('tabs-only    ', CHAR(9)//CHAR(9), 1)
        CALL one('minus-space  ', '- 5', 1)
        CALL one('plus-digit   ', '+0 5', 2)
        CALL one('many-digits  ', '0000000000000000042 Name', 1)
    END SUBROUTINE p
END PROGRAM fortran_io_probe2
