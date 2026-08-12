! Does gfortran's `x**2.0` (REAL exponent) equal `x*x` bit-for-bit, at the
! campaign's own flags?  NotchFilter writes `K**2.0` and `omega**2.0` five
! times; a C++ transcription has to choose between std::pow(x,2.0) and x*x,
! and the choice is only free if they agree on every bit.
!
! Prints the raw IEEE bits of both, plus the values the simulation actually
! uses: K = 2/DT with DT = 0.0125, omega = 1.0.
PROGRAM pow_two_probe
    IMPLICIT NONE
    REAL(8) :: x, a, b
    INTEGER(8) :: ia, ib
    INTEGER :: i, ndiff
    REAL(8), PARAMETER :: vals(10) = [ 160.0D0, 1.0D0, 0.25D0, 3.141592653589793D0, &
        1.0D-7, 1.0D7, 0.1D0, 7.0D0/3.0D0, 1.7976931348623157D300, 2.2250738585072014D-300 ]
    ndiff = 0
    DO i = 1, 10
        x = vals(i)
        a = x**2.0
        b = x*x
        ia = TRANSFER(a, ia)
        ib = TRANSFER(b, ib)
        WRITE(*,'(A,ES23.16,A,Z16.16,A,Z16.16,A,L1)') 'x=', x, '  x**2.0=', ia, '  x*x=', ib, '  same=', ia == ib
        IF (ia /= ib) ndiff = ndiff + 1
    END DO
    ! and a sweep over a pseudo-random spread
    x = 1.0D0
    DO i = 1, 200000
        x = MOD(x*1.0000173D0 + 0.000131D0, 1000.0D0) + 1.0D-6
        a = x**2.0
        b = x*x
        ia = TRANSFER(a, ia); ib = TRANSFER(b, ib)
        IF (ia /= ib) ndiff = ndiff + 1
    END DO
    WRITE(*,'(A,I0,A)') 'DIFFERING BIT PATTERNS: ', ndiff, ' of 200010'
END PROGRAM pow_two_probe
