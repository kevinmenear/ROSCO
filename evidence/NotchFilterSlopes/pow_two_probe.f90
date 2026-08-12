! Does gfortran's `x**2.0` (REAL exponent) equal `x*x` bit-for-bit, at the
! campaign's own flags?  NotchFilterSlopes writes `DT**2.0` and
! `CornerFreq_**2.0` three times each; a C++ transcription has to choose
! between std::pow(x,2.0) and x*x, and the choice is only free if they agree
! on every bit.
!
! Unit #13 measured the same question for NotchFilter's `K**2.0`/`omega**2.0`.
! Re-run here on THIS unit's own operands rather than cited: the named values
! are the ones the live call site hands over -- DT = 0.0125 s, Damp = 0.7, and
! CornerFreq_ = LocalVar%RotSpeedF, a rotor speed in rad/s that MOVES call to
! call (this unit passes Moving = .TRUE.), so the coefficient block runs every
! timestep on a different frequency rather than once at initialisation.
PROGRAM pow_two_probe
    IMPLICIT NONE
    REAL(8) :: x, a, b
    INTEGER(8) :: ia, ib
    INTEGER :: i, ndiff
    REAL(8), PARAMETER :: vals(10) = [ 0.0125D0, 0.7D0, 0.0D0, 0.8975979010256552D0, &
        1.2566370614359172D0, 6.283185307179586D0, 1.0D-7, 1.0D7, &
        1.7976931348623157D300, 2.2250738585072014D-300 ]
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
