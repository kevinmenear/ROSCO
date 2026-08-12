! interp1d's SIZE(xData) /= SIZE(yData) branch, transcribed exactly, in
! isolation. The two statements are Functions.f90:155-157 of the clean tree:
!
!     ErrVar%ErrMsg  = ' xData and yData are not the same size'
!     WRITE(ErrVar%ErrMsg,"(A,I2,A,I2,A)") " SIZE(xData) =", SIZE(xData), &
!     ' and SIZE(yData) =', SIZE(yData),' are not the same'
!
! ErrMsg is CHARACTER(:), ALLOCATABLE, so the assignment REALLOCATES it to the
! literal's length. The internal WRITE that follows formats a longer record
! into it. This program measures what gfortran does with that, so that the
! translation's divergence is stated against a measurement rather than an
! argument.
PROGRAM size_mismatch_probe
  CHARACTER(:), ALLOCATABLE :: msg
  INTEGER :: nx, ny
  nx = 3; ny = 5
  msg = ' xData and yData are not the same size'
  PRINT *, 'LEN after the assignment =', LEN(msg)
  PRINT *, 'characters the WRITE formats =', &
       LEN(" SIZE(xData) =") + 2 + LEN(' and SIZE(yData) =') + 2 + &
       LEN(' are not the same')
  WRITE(msg,"(A,I2,A,I2,A)") " SIZE(xData) =", nx, &
       ' and SIZE(yData) =', ny,' are not the same'
  PRINT *, 'LEN after the WRITE =', LEN(msg)
  PRINT *, '[', msg, ']'
END PROGRAM
