! Format-fidelity control for translations/ROSCO_IO/debug.cpp.
!
! The whole of `Debug`'s observable output is Fortran formatted records, so the
! translation stands or falls on reproducing four edit descriptors. This writes
! them from gfortran, with the same flags the controller is built with; the C++
! side (fmt_probe.cpp) writes the same values through the translation's own
! helpers, and the two files are compared byte for byte.
!
! P10: the value list is printed with the file, so "0 differences" names the set
! it compared. It includes the two clamp boundaries the reference imposes
! (1E-99, 1E+99), a signed zero, a subnormal, both signs, and the F-field
! overflow the status line can actually reach.
PROGRAM fmt_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: DP = KIND(1.0D0)
    INTEGER, PARAMETER :: N = 26
    REAL(DP) :: v(N)
    INTEGER :: i, u

    v = (/ 0.0D0, -0.0D0, 1.0D0, -1.0D0, 0.5D0, &
           1.0D-99, 1.0D+99, -1.0D-99, -1.0D+99, 9.99999D0, &
           1.234567890123D5, -1.234567890123D-5, 3.14159265358979D0, 1.0D-308, 1.0D-320, &
           123456.789D0, -123456.789D0, 1.0D0/3.0D0, 2.0D0/3.0D0, 1.0D10, &
           -1.0D10, 9.999995D-1, 1.0D-1, 6.02214076D23, -6.02214076D-23, &
           99999.999999D0 /)

    OPEN(NEWUNIT=u, FILE='fmt_probe.f.out')

    ! (F20.5,TR5,159(ES20.5E2,TR5:)) -- the record FmtDat writes, one value per
    ! record so a single mismatch names its own value.
    DO i = 1, N
        WRITE(u, "(F20.5,TR5,159(ES20.5E2,TR5:))") v(i), v(i), -v(i)
    END DO

    ! (99(a20,TR5:)) -- the heading record, including the CHARACTER(15)
    ! truncation the reference performs on five of its literals.
    WRITE(u, '(99(a20,TR5:))') 'Time', &
        (/ CHARACTER(15) :: 'WE_Cp', 'NacHeadingTarget', 'SU_LoadStageStartTime', '[-]' /)

    ! The status line's narrow F fields, where the asterisk-overflow arm is
    ! reachable.
    DO i = 1, N
        WRITE(u, 100) v(i)*9.5492966D0, v(i)*57.2957795130D0, v(i)/1000.0D0, v(i)
100     FORMAT('Generator speed: ', f6.1, ' RPM, Pitch angle: ', f5.1, &
               ' deg, Power: ', f7.1, ' kW, Est. wind Speed: ', f5.1, ' m/s')
    END DO

    ! List-directed write of a character expression -- the 'Generated on' line.
    WRITE(u, *) 'Generated on '//'01-Jan-2026'//' at '//'00:00:00'//' using ROSCO-'//'2.10.1'

    CLOSE(u)
END PROGRAM fmt_probe
