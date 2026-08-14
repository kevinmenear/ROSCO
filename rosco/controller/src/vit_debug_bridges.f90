! VIT production bridge for Debug's two wall-clock callees.
!
! WHY THIS FILE EXISTS.
!
! `plan.json` declares `CurDate` and `CurTime` PERMANENT BRIDGES for unit
! `Debug`. They are the two callees in that unit that are not functions of its
! arguments at all:
!
!     CALL DATE_AND_TIME ( CDate )     ! ROSCO_Helpers.f90, CurDate
!     CALL DATE_AND_TIME ( TIME=CTime )!                     CurTime
!
! Their result depends on when the program runs, so nothing in this campaign can
! compare a translation of them against the reference -- the reference does not
! give the same answer twice. Re-implementing them in C++ would also walk
! straight into a defect the check registry already names: `strftime`'s `%b` is
! locale-dependent, while CurDate's month name comes from a hardcoded English
! table. So they stay Fortran and are CALLED, which is X1: a callee is crossed
! through a bridge, never reproduced in its caller.
!
! WHAT CROSSES. Each Fortran function returns a fixed-width, blank-padded
! CHARACTER value -- CHARACTER(11) 'dd-mmm-yyyy' and CHARACTER(8) 'hh:mm:ss'.
! Neither is NUL-terminated, and the C++ side writes exactly those 11 and 8
! bytes into the record it is building, so no terminator is wanted. The buffers
! are C_CHAR arrays of the declared length: the length is a compile-time
! property of the callee, so it does not have to be passed.
!
! The reference calls these ONLY from inside `IF (CntrPar%LoggingLevel > n)`
! initialisation arms, so the bridge is entered at most three times per
! simulation.

MODULE vit_debug_bridges

    USE, INTRINSIC :: ISO_C_BINDING
    USE ROSCO_Helpers, ONLY : CurDate, CurTime

    IMPLICIT NONE

CONTAINS

    ! CurDate() -- CHARACTER(11), 'dd-mmm-yyyy'
    SUBROUTINE vit_curdate_c(CurDate_result) BIND(C, name='vit_curdate_c')
        CHARACTER(KIND=C_CHAR), INTENT(OUT) :: CurDate_result(11)
        CHARACTER(11) :: s
        INTEGER :: i

        s = CurDate()
        DO i = 1, 11
            CurDate_result(i) = s(i:i)
        END DO
    END SUBROUTINE vit_curdate_c

    ! CurTime() -- CHARACTER(8), 'hh:mm:ss'
    SUBROUTINE vit_curtime_c(CurTime_result) BIND(C, name='vit_curtime_c')
        CHARACTER(KIND=C_CHAR), INTENT(OUT) :: CurTime_result(8)
        CHARACTER(8) :: s
        INTEGER :: i

        s = CurTime()
        DO i = 1, 8
            CurTime_result(i) = s(i:i)
        END DO
    END SUBROUTINE vit_curtime_c

END MODULE vit_debug_bridges
