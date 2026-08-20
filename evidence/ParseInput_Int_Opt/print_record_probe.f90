! The REFERENCE half: gfortran's own record for this unit's PRINT statement.
PROGRAM print_record_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: N = 6
    CHARACTER(200) :: VarName
    INTEGER(4) :: Variable
    INTEGER :: i
    DO i = 1, N
        CALL cse(i, VarName, Variable)
        PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )// &
            " in input file.  Using default value of ", Variable
    END DO
CONTAINS
    SUBROUTINE cse(k, vn, v)
        INTEGER, INTENT(IN) :: k
        CHARACTER(*), INTENT(OUT) :: vn
        INTEGER(4), INTENT(OUT) :: v
        SELECT CASE (k)
        CASE (1); vn = 'PC_KP';   v = 0
        CASE (2); vn = 'X';       v = 0
        CASE (3); vn = '';        v = 0
        CASE (4); vn = 'WE_Mode'; v = -2147483647 - 1
        CASE (5); vn = 'A';       v = 2147483647
        CASE (6); vn = REPEAT('Z', 60); v = 7
        END SELECT
    END SUBROUTINE cse
END PROGRAM print_record_probe
