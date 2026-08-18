! What gfortran ACTUALLY does, for the three runtime behaviours this unit has
! that no amount of reading the standard settles:
!
!   1. `READ (Line,*,IOSTAT=s) Ary`  -- list-directed input of REAL(8) from an
!      internal file, for the sign of `s`, the values stored, and WHICH items
!      are left unchanged when it stops early.
!   2. `PRINT *, "..."//TRIM(ParamName)//"...", Ary, "]"` -- the record bytes.
!   3. `WRITE (UnEc,*) LineNum, Tab, ParamName, Tab, Ary` -- the record bytes.
!
! Built with the campaign's own translation-phase flags. Output goes to stdout
! with `|` markers around each record so trailing blanks are visible.
PROGRAM fortran_io_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = 8
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
        REAL(DbKi), ALLOCATABLE :: Ary(:)
        INTEGER :: s, i
        CHARACTER(40) :: b

        Line = text
        ALLOCATE(Ary(n))
        ! A recognisable sentinel, so "left unchanged" is distinguishable from
        ! "assigned zero".
        DO i = 1, n
            Ary(i) = -987.654_DbKi
        END DO
        s = 0
        READ (Line, *, IOSTAT=s) Ary
        WRITE (*, '(A,A,A,I2,A,I6,A)', ADVANCE='NO') &
            'READ  ', tag, '  n=', n, '  iostat=', s, '  ->'
        DO i = 1, n
            WRITE (b, '(ES24.17)') Ary(i)
            WRITE (*, '(A,A)', ADVANCE='NO') ' ', TRIM(ADJUSTL(b))
        END DO
        WRITE (*, '(A)') ''
        DEALLOCATE(Ary)
    END SUBROUTINE one_read

    SUBROUTINE read_probe()
        WRITE (*,'(A)') '--- 1. list-directed READ from an internal file ---'
        CALL one_read('plain-2      ', '9.120        11.400       ! IPC_Vramp  x', 2)
        CALL one_read('plain-1      ', '0.0000              ! F_NotchFreqs', 1)
        CALL one_read('short        ', '1.5   NameHere', 2)
        CALL one_read('empty        ', '   ', 1)
        CALL one_read('name-first   ', 'NameHere 1.0', 1)
        CALL one_read('commas       ', '1.0,2.0,3.0 Name', 3)
        CALL one_read('null-value   ', '1.0, ,3.0 Name', 3)
        CALL one_read('repeat       ', '3*7.5 Name', 3)
        CALL one_read('repeat-null  ', '2* 8.5 Name', 3)
        CALL one_read('slash        ', '1.0 / 2.0 Name', 3)
        CALL one_read('slash-first  ', '/ 1.0 2.0', 2)
        CALL one_read('d-exponent   ', '1.5d2 2.5D-1 Name', 2)
        CALL one_read('q-exponent   ', '1.5q2 Name', 1)
        CALL one_read('bare-exponent', '1.5+2 Name', 1)
        CALL one_read('int-form     ', '3 4 Name', 2)
        CALL one_read('leading-dot  ', '.5 Name', 1)
        CALL one_read('trailing-dot ', '5. Name', 1)
        CALL one_read('infinity     ', 'Infinity -inf Name', 2)
        CALL one_read('nan          ', 'NaN Name', 1)
        CALL one_read('tab-sep      ', '1.0'//Tab//'2.0 Name', 2)
        CALL one_read('bang         ', '! Name', 1)
        CALL one_read('partial-fail ', '1.0 2.0 bad 4.0', 4)
        CALL one_read('bad-first    ', 'bad 2.0', 2)
        CALL one_read('runout       ', '1.0 2.0', 4)
        CALL one_read('quoted       ', "'1.0' 2.0", 2)
        CALL one_read('semicolon    ', '1.0;2.0', 2)
        CALL one_read('star-alone   ', '*1.0', 1)
        CALL one_read('zero-repeat  ', '0*1.0 2.0', 2)
        CALL one_read('sign-only    ', '+ 1.0', 1)
        CALL one_read('two-dots     ', '1.0.0 2.0', 1)
        CALL one_read('exp-no-digit ', '1.0e Name', 1)
    END SUBROUTINE read_probe

    SUBROUTINE print_probe()
        CHARACTER(MaxParamLength) :: ParamName
        REAL(DbKi), ALLOCATABLE :: Ary(:)
        WRITE (*,'(A)') '--- 2. PRINT *, <char>, Ary, "]"   (Ary is all zero here) ---'
        ParamName = 'PC_GS_angles'
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
    END SUBROUTINE print_probe

    SUBROUTINE write_probe()
        CHARACTER(MaxParamLength) :: ParamName
        REAL(DbKi), ALLOCATABLE :: Ary(:)
        INTEGER :: LineNum
        WRITE (*,'(A)') '--- 3. WRITE (UnEc,*) LineNum, Tab, ParamName, Tab, Ary ---'
        ParamName = 'IPC_Vramp'
        LineNum = 72
        ALLOCATE(Ary(2))
        Ary(1) = 9.12_DbKi
        Ary(2) = 11.4_DbKi
        OPEN(UNIT=77, FILE='probe_unec.txt', ACTION='WRITE')
        WRITE (77,*) LineNum, Tab, ParamName, Tab, Ary
        LineNum = 1
        Ary(1) = 0.0_DbKi
        Ary(2) = -1.0e-9_DbKi
        WRITE (77,*) LineNum, Tab, ParamName, Tab, Ary
        CLOSE(77)
    END SUBROUTINE write_probe

END PROGRAM fortran_io_probe
