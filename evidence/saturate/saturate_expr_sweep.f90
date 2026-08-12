! The whole unit, swept. Reads (inputValue, minValue, maxValue) triples and
! prints the BITS of REAL(MIN(MAX(inputValue,minValue),maxValue),DbKi) --
! the reference statement, transcribed character for character from
! Functions.f90, compiled with the campaign's own flags.
!
! saturate_expr_sweep.cpp prints the same column for the same triples. A
! disagreement count of 0 over a sweep that contains both signed zeros, both
! infinities, a NaN, denormals and the decade ladder is what licenses spelling
! Fortran's MAX/MIN as fmax/fmin rather than as a branch.
PROGRAM saturate_expr_sweep
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = 8
    REAL(DbKi) :: inputValue, minValue, maxValue, r
    INTEGER :: ios

    DO
        READ (*, *, IOSTAT=ios) inputValue, minValue, maxValue
        IF (ios /= 0) EXIT
        r = REAL(MIN(MAX(inputValue, minValue), maxValue), DbKi)
        WRITE (*, '(Z16.16)') TRANSFER(r, 0_8)
    END DO
END PROGRAM saturate_expr_sweep
