PROGRAM m
  IMPLICIT NONE
  REAL(8) :: F(3,3), P(3,3), H(1,3), K(3,1), R_m, S(1,1), I3(3,3)
  REAL(8) :: M1(3,3), M2(3,3), M3(3,3), Kn(3,1), Pn(3,3)
  INTEGER :: i, j, n
  INTEGER(8) :: b
  DO n = 1, 3
    DO j = 1, 3
      DO i = 1, 3
        F(i,j) = SIN(1.0D0*(i + 3*j + 9*n)) * 1.234D0
        P(i,j) = COS(2.0D0*(i + 5*j + 7*n)) * 0.577D0
      END DO
      K(j,1) = SIN(0.7D0*(j + 11*n))
      H(1,j) = 0.0D0
    END DO
    H(1,1) = 1.0D0
    R_m = 0.02D0
    DO j = 1, 3
      DO i = 1, 3
        I3(i,j) = 0.0D0
      END DO
      I3(j,j) = 1.0D0
    END DO
    M1 = MATMUL(F,P)
    M2 = MATMUL(P,TRANSPOSE(F))
    M3 = MATMUL(K*R_m, TRANSPOSE(K))
    S  = MATMUL(H, MATMUL(P,TRANSPOSE(H))) + R_m
    Kn = MATMUL(P,TRANSPOSE(H))/S(1,1)
    Pn = MATMUL(I3 - MATMUL(K,H), P)
    DO j = 1, 3
      DO i = 1, 3
        b = TRANSFER(M1(i,j), b); WRITE(*,'(A,3I2,A,Z16)') 'M1 ',n,i,j,' ',b
        b = TRANSFER(M2(i,j), b); WRITE(*,'(A,3I2,A,Z16)') 'M2 ',n,i,j,' ',b
        b = TRANSFER(M3(i,j), b); WRITE(*,'(A,3I2,A,Z16)') 'M3 ',n,i,j,' ',b
        b = TRANSFER(Pn(i,j), b); WRITE(*,'(A,3I2,A,Z16)') 'Pn ',n,i,j,' ',b
      END DO
      b = TRANSFER(Kn(j,1), b); WRITE(*,'(A,3I2,A,Z16)') 'Kn ',n,j, 1,' ',b
    END DO
    b = TRANSFER(S(1,1), b); WRITE(*,'(A,3I2,A,Z16)') 'S  ',n,1,1,' ',b
  END DO
END PROGRAM m
