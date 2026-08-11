! Differential driver. One scenario, three implementations, one output format.
!
! Scenario A is the LIVE usage, transcribed from ROSCO_IO.f90:1117-1134 and
! ReadSetParameters.f90:1577-1590: the caller ALLOCATEs the array first, then
! calls AddToList to append. Both ROSCO call sites do this, so the `else`
! branch never runs in ROSCO -- scenario B exercises it anyway, because a
! translation is of the function, not of its current callers.
!
! Built three times with -DIMPL_{ORIG,VIT,CFI}. Output is compared byte for
! byte against the ORIG run; ORIG is the oracle (P7).

PROGRAM probe
#if defined(IMPL_ORIG)
    USE AddToList_Orig, ONLY: AddToList
#elif defined(IMPL_VIT)
    USE AddToList_Vit, ONLY: AddToList
#elif defined(IMPL_CFI)
    USE AddToList_Cfi, ONLY: AddToList
#else
#error "define one of IMPL_ORIG / IMPL_VIT / IMPL_CFI"
#endif
    IMPLICIT NONE
    INTEGER(4), DIMENSION(:), ALLOCATABLE :: list
    INTEGER :: i

    ! ---- Scenario A: allocated, then appended to (the ROSCO path) ----
    ALLOCATE(list(4))
    DO i = 1, 4
        list(i) = i
    END DO
    CALL report('A0', list)

    CALL AddToList(list, 91)
    CALL report('A1', list)

    CALL AddToList(list, 92)
    CALL report('A2', list)

    CALL AddToList(list, 17)
    CALL report('A3', list)

    DEALLOCATE(list)

    ! ---- Scenario B: unallocated on entry (the `else` branch) ----
#if defined(IMPL_VIT)
    ! VIT's wrapper declares `list` non-allocatable and evaluates SIZE(list)
    ! before the call, so an unallocated actual cannot legally be passed to it.
    ! Not skipped for convenience -- it does not compile as a call.
    WRITE(*,'(A)') 'B0 UNRUNNABLE the generated wrapper cannot accept an unallocated actual'
#else
    CALL report_unalloc('B0', list)
    CALL AddToList(list, 5)
    CALL report('B1', list)
    CALL AddToList(list, 6)
    CALL report('B2', list)
    DEALLOCATE(list)
#endif

CONTAINS

    SUBROUTINE report(tag, a)
        CHARACTER(*), INTENT(IN) :: tag
        INTEGER(4), DIMENSION(:), ALLOCATABLE, INTENT(IN) :: a
        CHARACTER(4096) :: buf, prev
        INTEGER :: k
        IF (.NOT. ALLOCATED(a)) THEN
            WRITE(*,'(A,A)') tag, ' UNALLOCATED'
            FLUSH(6)
            RETURN
        END IF
        buf = ''
        DO k = LBOUND(a,1), UBOUND(a,1)
            prev = buf
            WRITE(buf,'(A,1X,I0)') TRIM(prev), a(k)
        END DO
        WRITE(*,'(A," size=",I0," lb=",I0," ub=",I0," values:",A)') &
            tag, SIZE(a), LBOUND(a,1), UBOUND(a,1), TRIM(buf)
        FLUSH(6)
    END SUBROUTINE report

    SUBROUTINE report_unalloc(tag, a)
        CHARACTER(*), INTENT(IN) :: tag
        INTEGER(4), DIMENSION(:), ALLOCATABLE, INTENT(IN) :: a
        CALL report(tag, a)
    END SUBROUTINE report_unalloc

END PROGRAM probe
