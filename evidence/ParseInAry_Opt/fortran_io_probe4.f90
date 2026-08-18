! Fourth probe. The semicolon's ADJACENCY -- which separator preceded it --
! after the third probe showed `1;2` and `1 ;2` give different answers.
PROGRAM fortran_io_probe4
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
        CALL one('two-blanks   ', '1  ;2', 2)
        CALL one('tab-then-semi', '1'//CHAR(9)//';2', 2)
        CALL one('comma-sp-semi', '1 , ;2', 2)
        CALL one('null-then-semi', '1,,;3', 3)
        CALL one('rep-then-semi', '2* ;3', 3)
        CALL one('semi-item3-comma', '1,2 ;3', 3)
        CALL one('semi-after-3blank', '1   ;', 2)
        CALL one('semi-last-item', '1 2 ;', 2)
        CALL one('blank-semi-first', '  ;1', 1)
        CALL one('semi-then-semi', '1 ;;2', 2)
        CALL one('slash-then-semi', '1 /;2', 2)
        CALL one('semi-n3-first', '1 ;2 3', 3)
    END SUBROUTINE p
END PROGRAM fortran_io_probe4
