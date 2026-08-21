PROGRAM p
  IMPLICIT NONE
  REAL(8) :: x, y
  INTEGER :: i
  INTEGER(8) :: b
  DO i = 1, 8
     x = 1.0D0 + 0.37D0*i + 0.001D0*i*i
     y = x**3.0
     b = TRANSFER(y, b)
     WRITE(*,'(A,ES23.17,A,Z16)') 'F90 x**3.0 = ', y, '  bits=', b
     y = x*x*x
     b = TRANSFER(y, b)
     WRITE(*,'(A,ES23.17,A,Z16)') 'F90 x*x*x  = ', y, '  bits=', b
  END DO
END PROGRAM p
