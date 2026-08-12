! Does gfortran expand an INTEGER exponent by repeated multiplication, at this
! campaign's own flags?  `sigma` writes `(x0-x1)**3` four times, `x0**2` once,
! `x**3` and `x**2` once each, and a C++ transcription has to choose between
! `std::pow(v, 3.0)` and `v*v*v`.  Unit #13 measured the REAL exponent `2.0`
! and the check registry states the rule for INTEGER ones; this measures the
! exponents this unit actually has, because a rule read is not a rule run.
!
! Bits, not values: TRANSFER to INTEGER(8) and compare the patterns, so that
! +0.0 and -0.0 cannot pass for each other.
!
!   gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off -O3
PROGRAM int_pow_probe
    IMPLICIT NONE
    REAL(8) :: v, a, b
    INTEGER(8) :: ia, ib
    INTEGER :: i, n2, n3, ntot
    ! The nine special values, then a spread.  0.0 and -0.0 are in here on
    ! purpose: `**3` preserves the sign of zero and so does v*v*v, and unit #14
    ! established that a signed zero is where two spellings of the same
    ! arithmetic part company.
    REAL(8), PARAMETER :: vals(11) = [ 0.0D0, -0.0D0, 1.0D0, -1.0D0, 2.28D0, &
        -2.28D0, 9.12D0, 11.4D0, 3.141592653589793D0, 1.0D-160, 1.0D160 ]

    n2 = 0; n3 = 0; ntot = 0

    DO i = 1, 11
        v = vals(i)
        a = v**2;  b = v*v
        ia = TRANSFER(a, ia); ib = TRANSFER(b, ib)
        IF (ia /= ib) n2 = n2 + 1
        WRITE(*,'(A,ES23.16,A,Z16.16,A,Z16.16,A,L1)') &
            'v=', v, '  v**2=', ia, '  v*v=', ib, '  same=', ia == ib
        a = v**3;  b = v*v*v
        ia = TRANSFER(a, ia); ib = TRANSFER(b, ib)
        IF (ia /= ib) n3 = n3 + 1
        WRITE(*,'(A,ES23.16,A,Z16.16,A,Z16.16,A,L1)') &
            'v=', v, '  v**3=', ia, '  v*v*v=', ib, '  same=', ia == ib
        ntot = ntot + 2
    END DO

    ! A spread, including the (x0-x1) shape this unit divides by.
    v = 1.0D0
    DO i = 1, 30000
        v = MOD(v*1.0000173D0 + 0.000131D0, 200.0D0) - 100.0D0
        a = v**2;  b = v*v
        ia = TRANSFER(a, ia); ib = TRANSFER(b, ib)
        IF (ia /= ib) n2 = n2 + 1
        a = v**3;  b = v*v*v
        ia = TRANSFER(a, ia); ib = TRANSFER(b, ib)
        IF (ia /= ib) n3 = n3 + 1
        ntot = ntot + 2
    END DO

    WRITE(*,'(A,I0)')    'compared bit patterns:            ', ntot
    WRITE(*,'(A,I0)')    'v**2 differing from v*v:          ', n2
    WRITE(*,'(A,I0)')    'v**3 differing from v*v*v:        ', n3
END PROGRAM int_pow_probe
