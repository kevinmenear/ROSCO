! Measured, not read. `vit interface` emits
!     NonDecreasing_result = nondecreasing_c(Array, SIZE(Array))
! assigning an INTEGER(C_INT) to a LOGICAL. gfortran accepts it as an
! EXTENSION (one warning, exit 0) -- so the question that decides whether the
! bridge is exact is what the conversion DOES, and a bit-copy and a
! normalisation are not the same answer for a return value of 2.
!
! Run:  gfortran logical_result_conversion_probe.f90 -o probe && ./probe
! Measured 2026-08-11, gfortran in the vit-dev container:
!            0  F  T            0
!            1  T  F            1
!            2  T  F            1
!           -1  T  F            1
!          256  T  F            1
! It NORMALISES: nonzero -> .TRUE. with TRANSFER(L,0) == 1, zero -> .FALSE..
! Returning 1/0 from the C++ is therefore exact, and no value the translation
! can return produces a LOGICAL that `.NOT.` reads inconsistently.
PROGRAM p
  IMPLICIT NONE
  INTEGER :: vals(5), i
  LOGICAL :: L
  vals = (/0, 1, 2, -1, 256/)
  DO i = 1, 5
     L = conv(vals(i))
     PRINT '(I12,1X,L2,1X,L2,1X,I12)', vals(i), L, .NOT. L, TRANSFER(L, 0)
  END DO
CONTAINS
  LOGICAL FUNCTION conv(n)
    INTEGER, INTENT(IN) :: n
    conv = n
  END FUNCTION
END PROGRAM
