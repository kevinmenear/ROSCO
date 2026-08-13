! Does the REFERENCE terminate on its own declared domain?
!
! `unwrap`'s two guards are `DO WHILE` loops whose only progress is
! `y(i:) = y(i:) +/- 2*PI`. Once the tail's magnitude is large enough that the
! addition rounds back to the same double, the guard's value never changes.
! The loop body below is copied VERBATIM from Functions.f90:538-546; the
! ErrVar tail plays no part in termination and is left out.
!
! Run under `timeout`: a hang is the measurement.
PROGRAM reference_termination_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = 8
    REAL(DbKi), PARAMETER :: PI = 3.14159265359
    REAL(DbKi) :: v, step
    CHARACTER(len=32) :: arg
    INTEGER :: mode

    CALL GET_COMMAND_ARGUMENT(1, arg)
    READ(arg, *) mode

    IF (mode == 0) THEN
        ! The threshold, found by doubling: the smallest power-of-two magnitude
        ! at which `v + 2*PI == v`, i.e. at which the loop can no longer move.
        v = 1.0_DbKi
        DO WHILE (v + 2*PI /= v)
            v = v * 2.0_DbKi
        END DO
        WRITE(*,'(A,E24.17)') 'smallest 2**k with v + 2*PI == v : ', v
        WRITE(*,'(A,E24.17)') '2**52 * 2*PI                     : ', &
            2.0_DbKi**52 * (2*PI)
        WRITE(*,'(A)') 'a y(i) at or above that magnitude cannot leave either loop'
    ELSE IF (mode == 1) THEN
        CALL spin(1.0e17_DbKi)     ! above the threshold: expected to HANG
    ELSE IF (mode == 2) THEN
        CALL spin(1.0e300_DbKi)    ! the R3 magnitude ladder's own rung
    ELSE IF (mode == 3) THEN
        CALL spin(1.0e3_DbKi)      ! inside _bounds' default: expected to STOP
    END IF

CONTAINS

    SUBROUTINE spin(big)
        REAL(DbKi), INTENT(IN) :: big
        REAL(DbKi) :: y(2)
        INTEGER(8) :: n
        y(1) = 0.0_DbKi
        y(2) = big
        n = 0
        ! Functions.f90:543-545, verbatim, for i = 2 and SIZE(x) = 2
        DO WHILE (y(2) - y(1) .GE. PI)
            y(2:2) = y(2:2) - 2 * PI
            n = n + 1
            IF (MOD(n, 100000000_8) == 0) THEN
                WRITE(*,'(A,I0,A,E24.17)') '  still looping after ', n, &
                    ' iterations, y(2) = ', y(2)
                FLUSH(6)
            END IF
        END DO
        WRITE(*,'(A,E24.17,A,I0,A)') 'TERMINATED for big = ', big, &
            ' after ', n, ' iterations'
    END SUBROUTINE

END PROGRAM
