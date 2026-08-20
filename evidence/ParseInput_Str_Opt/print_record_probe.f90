! The REFERENCE half: gfortran's own record for this unit's PRINT statement.
!
! The layout is TWO ADJACENT CHARACTER ITEMS, which no sibling in this family
! has -- #55 writes CHARACTER, an INTEGER array, then a CHARACTER; #56 writes
! CHARACTER then one REAL; #57 writes CHARACTER then one INTEGER. So the one
! libgfortran rule this record turns on is the one none of them exercised, and
! the translation DERIVES it (a CHARACTER after a CHARACTER takes no separator
! when DELIM=NONE, which is the default) before this probe prices it.
!
! `Variable = 'unused'` is the statement immediately above the PRINT, so the
! second item is `TRIM('unused')` TRUNCATED TO LEN(Variable) -- which is the
! only input this record has, and is why the lengths below straddle six.
PROGRAM print_record_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: N = 9
    CHARACTER(:), ALLOCATABLE :: vn, var
    INTEGER :: i, L

    DO i = 1, N
        CALL cse(i, vn, L)
        IF (ALLOCATED(var)) DEALLOCATE(var)
        ALLOCATE(CHARACTER(L) :: var)
        var = REPEAT('~', L)
        CALL body(vn, var)
    END DO

CONTAINS

    ! The two statements under test, in the reference's own context: `Variable`
    ! is a `CHARACTER(*), INTENT(INOUT)` dummy.
    SUBROUTINE body(VarName, Variable)
        CHARACTER(*), INTENT(IN   ) :: VarName
        CHARACTER(*), INTENT(INOUT) :: Variable
        Variable = 'unused'     ! Default of string input is unused for now
        PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )// &
            " in input file.  Using default value of ", TRIM(Variable)
    END SUBROUTINE body

    SUBROUTINE cse(k, vn_, L_)
        INTEGER,                   INTENT(IN   ) :: k
        CHARACTER(:), ALLOCATABLE, INTENT(INOUT) :: vn_
        INTEGER,                   INTENT(  OUT) :: L_
        SELECT CASE (k)
        CASE (1); vn_ = 'PerfFileName';    L_ = 1024   ! five shipped callers
        CASE (2); vn_ = 'ZMQ_CommAddress'; L_ =  256   ! the sixth
        CASE (3); vn_ = 'X';               L_ =    6   ! exactly 'unused'
        CASE (4); vn_ = 'X';               L_ =    5   ! truncated to 'unuse'
        CASE (5); vn_ = 'X';               L_ =    1   ! truncated to 'u'
        CASE (6); vn_ = 'X';               L_ =    0   ! a ZERO-LENGTH item
        CASE (7); vn_ = '';                L_ =    7   ! an empty name
        CASE (8); vn_ = REPEAT('Z', 60);   L_ =    7
        CASE (9); vn_ = 'DLL_ProcName';    L_ = 1024
        END SELECT
    END SUBROUTINE cse

END PROGRAM print_record_probe
