! What `PI` IS on this toolchain -- bits, not a printed decimal.
! Constants.f90:24 declares  REAL(DbKi), PARAMETER :: PI = 3.14159265359
PROGRAM pi_literal_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = 8
    REAL(DbKi), PARAMETER :: PI = 3.14159265359
    REAL(DbKi) :: two
    two = 2 * PI
    WRITE(*,'(A,Z16.16)') 'PI      bits = ', TRANSFER(PI, 0_8)
    WRITE(*,'(A,Z16.16)') '2*PI    bits = ', TRANSFER(two, 0_8)
    WRITE(*,'(A,Z16.16)') '-PI     bits = ', TRANSFER(-PI, 0_8)
    WRITE(*,'(A,E24.17)') 'PI      dec  = ', PI
    WRITE(*,'(A,E24.17)') '2*PI    dec  = ', two
END PROGRAM
