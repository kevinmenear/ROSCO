PROGRAM xfer
  USE, INTRINSIC :: ISO_C_BINDING
  IMPLICIT NONE
  INTEGER :: n, k
  DO n = 1, 6
    BLOCK
      CHARACTER(KIND=C_CHAR) :: buf(n)
      CHARACTER(100), PARAMETER :: root = 'external_control'
      CHARACTER(KIND=C_CHAR) :: out(LEN_TRIM(root)+1)
      buf = 'Z'
      buf = TRANSFER( C_NULL_CHAR, buf )
      WRITE(*,'(A,I0,A)', ADVANCE='NO') 'n=', n, ' bytes:'
      DO k = 1, n
        WRITE(*,'(1X,I0)', ADVANCE='NO') ICHAR(buf(k))
      END DO
      WRITE(*,*)
      out = TRANSFER( TRIM(root)//C_NULL_CHAR, out )
      WRITE(*,'(A,I0,A)', ADVANCE='NO') '  outname size=', SIZE(out), ' :'
      DO k = 1, SIZE(out)
        WRITE(*,'(1X,I0)', ADVANCE='NO') ICHAR(out(k))
      END DO
      WRITE(*,*)
    END BLOCK
  END DO
  PRINT *, 'SIZE(TRANSFER(C_NULL_CHAR, [character(kind=c_char)::])) test done'
END PROGRAM
