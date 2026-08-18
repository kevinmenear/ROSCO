! Fifth probe. The state the semicolon leaves behind, and the end-of-line
! separator, which behaves like a comma for the ';' test and not for the null.
PROGRAM fortran_io_probe5
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
        CALL one('semi-semi-n3 ', '1 ;;2 3', 3)
        CALL one('semi-sp-semi ', '1 ; ;2', 3)
        CALL one('semi-then-comma', '1 ;,2 3', 3)
        CALL one('comma-then-semi-n3', '1, ;2', 3)
        CALL one('comma-blank-semi', '1 , ;2 3', 3)
        CALL one('lf-then-semi ', '1'//CHAR(10)//';2', 2)
        CALL one('semi-after-null-blank', '1 , 2 ;3', 3)
        CALL one('nl-sep-comma ', '1'//CHAR(10)//',2', 2)
    END SUBROUTINE p
END PROGRAM fortran_io_probe5
