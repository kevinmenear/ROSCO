! MACHINE-READABLE form of the two probes above, plus the corners each of the
! translation's branches has. It writes one line per record:
!
!     CASE <n> <iostat> <v1..vn>  LEN <m> <c1..cm>
!
! where the `c` are ICHAR codes of the SUPPLIED text -- the C++ replay
! reconstructs the record by blank-padding it to MaxLineLength, which is what
! `CHARACTER(MaxLineLength) :: Line` hands the reference's own READ.
!
! WHY THIS EXISTS. Unit #54's parser was written from a probe that a human read
! and transcribed, and the transcription was right; but the reference's answer
! for a record is a fact, and a translation that reproduces it should be able to
! SAY SO by running. `parser_conformance.cpp` replays every record below through
! `translations/ROSCO_Helpers/parseinary_opt.cpp`'s own `list_read_ints` and
! compares the iostat AND every element, sentinel included. It is a positive
! control on the one function of this unit that no other layer exercises more
! than a hundred times.
PROGRAM parser_conformance
    IMPLICIT NONE
    INTEGER, PARAMETER :: IntKi = 4
    INTEGER, PARAMETER :: MaxLineLength = 2048
    CHARACTER(1), PARAMETER :: T = CHAR(9)
    CHARACTER(1), PARAMETER :: LF = CHAR(10)
    CHARACTER(1), PARAMETER :: CR = CHAR(13)

    ! --- the shapes shipped input files have -------------------------------
    CALL one('3 4        ! F_NotchType  x', 2)
    CALL one('0                  ! Y_ControlMode', 1)
    CALL one('+7 -8 Name', 2)
    CALL one('007 Name', 1)
    ! --- stopping early -----------------------------------------------------
    CALL one('5   NameHere', 2)
    CALL one('   ', 1)
    CALL one('1 2', 4)
    CALL one('NameHere 1', 1)
    ! --- separators and null values -----------------------------------------
    CALL one('1,2,3 Name', 3)
    CALL one('1, ,3 Name', 3)
    CALL one('1'//T//'2 Name', 2)
    CALL one('1,,3 Name', 3)
    CALL one('1, Name', 2)
    CALL one('1,', 2)
    CALL one(',1 2', 2)
    CALL one(',,1 2', 3)
    ! --- repeat counts ------------------------------------------------------
    CALL one('3*7 Name', 3)
    CALL one('2* 8 Name', 3)
    CALL one('0*1 2', 2)
    CALL one('*1', 1)
    CALL one('2*-5 Name', 2)
    CALL one('2*+5 Name', 2)
    CALL one('5*7 Name', 3)
    CALL one('3*', 3)
    CALL one('2*,9 Name', 3)
    CALL one('2*3.5 Name', 2)
    CALL one('02*7 Name', 2)
    CALL one('99999999999*7 Name', 2)
    CALL one('2147483647*7 Name', 2)
    CALL one('2147483648*7 Name', 2)
    CALL one('200000000*7 Name', 2)
    CALL one('200000001*7 Name', 2)
    CALL one('199999999*7 Name', 2)
    CALL one('-2*7 Name', 2)
    CALL one('2**7 Name', 2)
    CALL one('1*', 2)
    ! --- the terminator -----------------------------------------------------
    CALL one('1 / 2 Name', 3)
    CALL one('/ 1 2', 2)
    CALL one('1/2 Name', 3)
    CALL one('1 /', 3)
    CALL one("1' 2", 2)
    CALL one('1! 2', 2)
    CALL one('1'//CR//' 2', 2)
    CALL one('1'//LF//' 2', 2)
    CALL one('1'//LF//',2', 2)
    ! --- REAL text into an INTEGER list -------------------------------------
    CALL one('1.5 2 Name', 2)
    CALL one('5. Name', 1)
    CALL one('.5 Name', 1)
    CALL one('. 1', 1)
    CALL one('-. 1', 1)
    CALL one('1.0.0 2', 1)
    CALL one('1e2 Name', 1)
    CALL one('1d2 Name', 1)
    CALL one('1+2 Name', 1)
    CALL one('. 5', 2)
    CALL one('., 5', 2)
    CALL one('./ 5', 2)
    CALL one('+. 5', 2)
    CALL one('5. 6', 2)
    CALL one('5., 6', 2)
    CALL one('e1 5', 2)
    CALL one('.e1 5', 2)
    ! --- junk ---------------------------------------------------------------
    CALL one('bad 2', 2)
    CALL one('1 2 bad 4', 4)
    CALL one('5abc 2', 2)
    CALL one('! Name', 1)
    CALL one("'1' 2", 2)
    CALL one('+ 1', 1)
    CALL one('- 1', 1)
    CALL one('- 5', 1)
    CALL one('+0 5', 2)
    ! --- range --------------------------------------------------------------
    CALL one('2147483647 Name', 1)
    CALL one('2147483648 Name', 1)
    CALL one('-2147483648', 1)
    CALL one('-2147483649 Name', 1)
    CALL one('99999999999999999999 2', 2)
    CALL one('1 99999999999999999999', 2)
    CALL one('2147483646', 1)
    CALL one('002147483648', 1)
    CALL one('-0000000000005', 1)
    CALL one('0000000000000000042 Name', 1)
    CALL one('     42 Name', 1)
    CALL one(T//T, 1)
    ! --- the semicolon, which is where the INTEGER reader and the REAL reader
    !     part company: unit #54 measured '1.0;2.0' as STORING 1.0, and '1;2'
    !     stores nothing -----------------------------------------------------
    CALL one(';', 1)
    CALL one(';1', 1)
    CALL one('1;2', 2)
    CALL one('1;2', 1)
    CALL one('1 ;2', 2)
    CALL one('1 ; 2', 2)
    CALL one('1; 2', 2)
    CALL one('1 2 ;3', 3)
    CALL one('1, ;2', 2)
    CALL one('1 ;', 2)
    CALL one(';1 2', 2)
    CALL one('1  ;2', 2)
    CALL one('1'//T//';2', 2)
    CALL one('1 , ;2', 2)
    CALL one('1,,;3', 3)
    CALL one('2* ;3', 3)
    CALL one('1,2 ;3', 3)
    CALL one('1   ;', 2)
    CALL one('1 2 ;', 2)
    CALL one('  ;1', 1)
    CALL one('1 ;;2', 2)
    CALL one('1 /;2', 2)
    CALL one('1 ;2 3', 3)
    CALL one('1 ;;2 3', 3)
    CALL one('1 ; ;2', 3)
    CALL one('1 ;,2 3', 3)
    CALL one('1, ;2', 3)
    CALL one('1 , ;2 3', 3)
    CALL one('1'//LF//';2', 2)
    CALL one('1 , 2 ;3', 3)

CONTAINS

    SUBROUTINE one(text, n)
        CHARACTER(*), INTENT(IN) :: text
        INTEGER, INTENT(IN) :: n
        CHARACTER(MaxLineLength) :: Line
        INTEGER(IntKi), ALLOCATABLE :: Ary(:)
        INTEGER :: s, i
        Line = text
        ALLOCATE(Ary(n))
        DO i = 1, n
            Ary(i) = -987654_IntKi
        END DO
        s = 0
        READ (Line, *, IOSTAT=s) Ary
        WRITE (*,'(A,I0,A,I0)',ADVANCE='NO') 'CASE ', n, ' ', s
        DO i = 1, n
            WRITE (*,'(A,I0)',ADVANCE='NO') ' ', Ary(i)
        END DO
        WRITE (*,'(A,I0)',ADVANCE='NO') ' LEN ', LEN(text)
        DO i = 1, LEN(text)
            WRITE (*,'(A,I0)',ADVANCE='NO') ' ', ICHAR(text(i:i))
        END DO
        WRITE (*,'(A)') ''
        DEALLOCATE(Ary)
    END SUBROUTINE one

END PROGRAM parser_conformance
