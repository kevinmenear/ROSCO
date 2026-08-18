PROGRAM p2
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = 8
    CALL one('sign-blank ', '+ 1.0', 2)
    CALL one('minus-blank', '- 1.0', 2)
    CALL one('dot-blank  ', '. 1.0', 2)
    CALL one('mdot-blank ', '-. 1.0', 2)
    CALL one('pdot-blank ', '+. 1.0', 2)
    CALL one('dot-slash  ', './\', 2)
    CALL one('mdot-slash ', '-./\', 2)
    CALL one('comma-mdot ', ',-./', 3)
    CALL one('sign-slash ', '+/1.0', 2)
    CALL one('dot-comma  ', '.,1.0', 2)
    CALL one('sign-comma ', '+,1.0', 2)
    CALL one('dot-only   ', '.', 2)
    CALL one('e-only     ', 'e1 1.0', 2)
    CALL one('dot-e      ', '.e1 1.0', 2)
    CALL one('mdot-e     ', '-.e 1.0', 2)
    CALL one('dotdot     ', '.. 1.0', 2)
    CALL one('dot-then-d ', '.d0 1.0', 2)
    CALL one('minus-only ', '-', 2)
    CALL one('plus-eor   ', '+', 2)
    CALL one('dot-eor    ', '.', 1)
CONTAINS
    SUBROUTINE one(tag, text, n)
        CHARACTER(*), INTENT(IN) :: tag, text
        INTEGER, INTENT(IN) :: n
        CHARACTER(2048) :: Line
        REAL(DbKi) :: A(n)
        INTEGER :: s, i
        CHARACTER(40) :: b
        Line = text
        A = -987.654_DbKi
        s = 0
        READ (Line, *, IOSTAT=s) A
        WRITE (*,'(A,A,A,I6,A)',ADVANCE='NO') 'READ ', tag, ' iostat=', s, ' ->'
        DO i = 1, n
            WRITE (b,'(ES24.17)') A(i)
            WRITE (*,'(A,A)',ADVANCE='NO') ' ', TRIM(ADJUSTL(b))
        END DO
        WRITE (*,'(A)') ''
    END SUBROUTINE one
END PROGRAM p2
