PROGRAM R
  IMPLICIT NONE
  INTEGER, PARAMETER :: DbKi = SELECTED_REAL_KIND(14,300)
  REAL(DbKi) :: v, seed
  INTEGER :: i, k
  OPEN(77, FILE="vals.bin", FORM="UNFORMATTED", ACCESS="STREAM", STATUS="REPLACE")
  DO k = -320, 308
     v = 1.23456789012345678_DbKi * (10.0_DbKi ** k)
     CALL emit(v)
     CALL emit(-v)
     v = 9.99999999999999889_DbKi * (10.0_DbKi ** k)
     CALL emit(v)
     v = 1.0_DbKi * (10.0_DbKi ** k)
     CALL emit(v)
  END DO
  CALL emit(0.0_DbKi)
  CALL emit(-0.0_DbKi)
  CALL emit(HUGE(v))
  CALL emit(-HUGE(v))
  CALL emit(TINY(v))
  CALL emit(EPSILON(v))
  CALL emit(0.1_DbKi)
  CALL emit(0.5_DbKi)
  CALL emit(1.0_DbKi)
  CALL emit(9.5_DbKi)
  seed = 0.6180339887498949_DbKi
  DO i = 1, 20000
     seed = MOD(seed * 9973.0_DbKi + 0.31830988618379_DbKi, 1.0_DbKi)
     k = MODULO(i, 44) - 22
     v = (seed - 0.5_DbKi) * (10.0_DbKi ** k)
     CALL emit(v)
  END DO
  CLOSE(77)
CONTAINS
  SUBROUTINE emit(x)
    REAL(DbKi), INTENT(IN) :: x
    WRITE(401,*) x
    WRITE(77) x
  END SUBROUTINE
END PROGRAM
