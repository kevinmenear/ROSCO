! What gfortran ACTUALLY does for the two runtime behaviours `ParseInAry_Opt`
! has that no reading of the standard settles. This is unit #54's probe with
! the ITEM TYPE CHANGED -- REAL(DbKi) to INTEGER(IntKi) -- because that is the
! only substantive difference between the two subroutines (their bodies are
! otherwise identical; `diff` of the two clean bodies is the routine name, the
! declaration of `Ary`, and whitespace).
!
!   1. `READ (Line,*,IOSTAT=s) Ary`  -- list-directed input of INTEGER(4) from
!      an internal file: the sign of `s`, the values stored, and WHICH items
!      are left unchanged when it stops early.
!   2. `PRINT *, "..."//TRIM(ParamName)//"...", Ary, "]"` -- the record bytes.
!   3. `WRITE (UnEc,*) LineNum, Tab, ParamName, Tab, Ary` -- the record bytes.
!      (Not translated; measured so the evidence can say what is not written.)
!
! A `-987654` sentinel fills every element, so "left unchanged" is
! distinguishable from "assigned zero" -- which is the distinction that cost
! unit #54 three cases of 13,674.
PROGRAM fortran_io_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: IntKi = 4
    INTEGER, PARAMETER :: MaxLineLength = 2048
    INTEGER, PARAMETER :: MaxParamLength = 200
    CHARACTER(1), PARAMETER :: Tab = CHAR(9)

    CALL read_probe()
    CALL print_probe()
    CALL write_probe()

CONTAINS

    SUBROUTINE one_read(tag, text, n)
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
    END SUBROUTINE one_read

    SUBROUTINE read_probe()
        WRITE (*,'(A)') '--- 1. list-directed READ of INTEGER(4) from an internal file ---'
        ! the shapes the shipped input files actually have
        CALL one_read('plain-2      ', '3 4        ! F_NotchType  x', 2)
        CALL one_read('plain-1      ', '0                  ! Y_ControlMode', 1)
        CALL one_read('signed       ', '+7 -8 Name', 2)
        CALL one_read('leading-zero ', '007 Name', 1)
        ! stopping early
        CALL one_read('short        ', '5   NameHere', 2)
        CALL one_read('empty        ', '   ', 1)
        CALL one_read('runout       ', '1 2', 4)
        CALL one_read('name-first   ', 'NameHere 1', 1)
        ! separators and null values
        CALL one_read('commas       ', '1,2,3 Name', 3)
        CALL one_read('null-value   ', '1, ,3 Name', 3)
        CALL one_read('tab-sep      ', '1'//Tab//'2 Name', 2)
        CALL one_read('semicolon    ', '1;2', 2)
        ! repeat counts
        CALL one_read('repeat       ', '3*7 Name', 3)
        CALL one_read('repeat-null  ', '2* 8 Name', 3)
        CALL one_read('zero-repeat  ', '0*1 2', 2)
        CALL one_read('star-alone   ', '*1', 1)
        ! terminator
        CALL one_read('slash        ', '1 / 2 Name', 3)
        CALL one_read('slash-first  ', '/ 1 2', 2)
        ! REAL text into an INTEGER list -- the whole point of this probe
        CALL one_read('real-form    ', '1.5 2 Name', 2)
        CALL one_read('trailing-dot ', '5. Name', 1)
        CALL one_read('leading-dot  ', '.5 Name', 1)
        CALL one_read('dot-alone    ', '. 1', 1)
        CALL one_read('dot-sign     ', '-. 1', 1)
        CALL one_read('two-dots     ', '1.0.0 2', 1)
        CALL one_read('exponent     ', '1e2 Name', 1)
        CALL one_read('d-exponent   ', '1d2 Name', 1)
        CALL one_read('bare-exponent', '1+2 Name', 1)
        ! junk
        CALL one_read('bad-first    ', 'bad 2', 2)
        CALL one_read('partial-fail ', '1 2 bad 4', 4)
        CALL one_read('digits-junk  ', '5abc 2', 2)
        CALL one_read('bang         ', '! Name', 1)
        CALL one_read('quoted       ', "'1' 2", 2)
        CALL one_read('sign-only    ', '+ 1', 1)
        CALL one_read('minus-only   ', '- 1', 1)
        ! range
        CALL one_read('int-max      ', '2147483647 Name', 1)
        CALL one_read('int-max-p1   ', '2147483648 Name', 1)
        CALL one_read('int-min      ', '-2147483648 Name', 1)
        CALL one_read('int-min-m1   ', '-2147483649 Name', 1)
        CALL one_read('huge-digits  ', '99999999999999999999 2', 2)
        CALL one_read('overflow-2nd ', '1 99999999999999999999', 2)
    END SUBROUTINE read_probe

    SUBROUTINE print_probe()
        CHARACTER(MaxParamLength) :: ParamName
        INTEGER(IntKi), ALLOCATABLE :: Ary(:)
        WRITE (*,'(A)') '--- 2. PRINT *, <char>, Ary, "]"   (Ary is all zero here) ---'
        ParamName = 'F_NotchType'
        ALLOCATE(Ary(3))
        Ary = 0
        WRITE (*,'(A)',ADVANCE='NO') 'PRINT3 |'
        PRINT *, "ROSCO Warning: Did not find correct size "//TRIM( ParamName )// &
                 " in input file.  Using default value of [", Ary, "]"
        DEALLOCATE(Ary)
        ALLOCATE(Ary(1))
        Ary = 0
        WRITE (*,'(A)',ADVANCE='NO') 'PRINT1 |'
        PRINT *, "ROSCO Warning: Did not find correct size "//TRIM( ParamName )// &
                 " in input file.  Using default value of [", Ary, "]"
        DEALLOCATE(Ary)
        ! and with values that exercise the field width, since `Ary = 0` is the
        ! only value the reference ever prints but a mutant need not be
        ALLOCATE(Ary(4))
        Ary(1) = 0; Ary(2) = -1; Ary(3) = 2147483647; Ary(4) = -2147483647
        WRITE (*,'(A)',ADVANCE='NO') 'PRINT4 |'
        PRINT *, "[", Ary, "]"
        DEALLOCATE(Ary)
    END SUBROUTINE print_probe

    SUBROUTINE write_probe()
        CHARACTER(MaxParamLength) :: ParamName
        INTEGER(IntKi), ALLOCATABLE :: Ary(:)
        INTEGER :: LineNum
        WRITE (*,'(A)') '--- 3. WRITE (UnEc,*) LineNum, Tab, ParamName, Tab, Ary ---'
        ParamName = 'F_NotchType'
        LineNum = 72
        ALLOCATE(Ary(2))
        Ary(1) = 1
        Ary(2) = -2
        OPEN(UNIT=77, FILE='probe_unec.txt', ACTION='WRITE')
        WRITE (77,*) LineNum, Tab, ParamName, Tab, Ary
        CLOSE(77)
    END SUBROUTINE write_probe

END PROGRAM fortran_io_probe
