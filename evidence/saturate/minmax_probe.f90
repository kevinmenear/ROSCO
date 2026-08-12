! What does gfortran's MIN/MAX actually DO at a signed zero and at a NaN?
!
! `saturate` is one statement -- REAL(MIN(MAX(inputValue,minValue),maxValue),DbKi)
! -- so the whole translation is the expansion of two intrinsics. Two spellings
! are equally plausible in C++ and they DISAGREE on exactly the inputs unit #14's
! signed-zero corpus block exists to produce:
!
!     MAX(a,b) as (a > b) ? a : b     MAX(-0.0, +0.0) = +0.0
!     MAX(a,b) as (b > a) ? b : a     MAX(-0.0, +0.0) = -0.0
!
! Values are READ AT RUNTIME so the compiler cannot fold them; a constant-folded
! probe measures gfortran's front end, and the shipped controller runs the back
! end. Bits are compared through TRANSFER, not the values, because +0.0 == -0.0.
PROGRAM minmax_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = 8
    REAL(DbKi) :: a, b
    INTEGER :: ios, n

    n = 0
    DO
        READ (*, *, IOSTAT=ios) a, b
        IF (ios /= 0) EXIT
        n = n + 1
        WRITE (*, '(A,I3,4(1X,Z16.16))') 'case', n, TRANSFER(a, 0_8), TRANSFER(b, 0_8), &
            TRANSFER(MAX(a, b), 0_8), TRANSFER(MIN(a, b), 0_8)
    END DO
END PROGRAM minmax_probe
