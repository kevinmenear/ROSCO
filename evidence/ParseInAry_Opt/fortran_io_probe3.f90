! Third probe. The semicolon systematically, the repeat-count ceiling, and
! what terminates a good value. Run under `docker exec vit-dev gfortran -O2`.
PROGRAM fortran_io_probe3
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
        WRITE (*,'(A)') '--- semicolon, systematically ---'
        CALL one('semi-only    ', ';', 1)
        CALL one('semi-1       ', ';1', 1)
        CALL one('semi-tight   ', '1;2', 2)
        CALL one('semi-sp-before', '1 ;2', 2)
        CALL one('semi-sp-both ', '1 ; 2', 2)
        CALL one('semi-sp-after', '1; 2', 2)
        CALL one('semi-n1      ', '1;2', 1)
        CALL one('semi-3rd     ', '1 2 ;3', 3)
        CALL one('semi-after-comma', '1, ;2', 2)
        CALL one('semi-eor     ', '1 ;', 2)
        WRITE (*,'(A)') '--- repeat-count bound ---'
        CALL one('rep-2147483647', '2147483647*7 Name', 2)
        CALL one('rep-2147483648', '2147483648*7 Name', 2)
        CALL one('rep-100000   ', '100000*7 Name', 2)
        CALL one('rep-negative ', '-2*7 Name', 2)
        CALL one('rep-star-star', '2**7 Name', 2)
        WRITE (*,'(A)') '--- terminator of a good value ---'
        CALL one('val-then-star', '1*', 2)
        CALL one('val-then-quote', "1' 2", 2)
        CALL one('val-then-bang', '1! 2', 2)
        CALL one('val-then-cr  ', '1'//CHAR(13)//' 2', 2)
        CALL one('val-then-lf  ', '1'//CHAR(10)//' 2', 2)
        WRITE (*,'(A)') '--- overflow boundary ---'
        CALL one('p2147483646  ', '2147483646', 1)
        CALL one('m2147483648  ', '-2147483648', 1)
        CALL one('p0021474836488', '002147483648', 1)
        CALL one('m0000000000005', '-0000000000005', 1)
    END SUBROUTINE p
END PROGRAM fortran_io_probe3
