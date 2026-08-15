! INPUT to evidence/interp2d/reference.size-mismatch-aborts.txt.
!
! interp2d's two size-mismatch branches, transcribed out of
! rosco/controller/src/Functions.f90 (clean source lines 198-203) with nothing
! added but the LEN report. The question is what the reference DOES on that
! input, and the answer decides whether the translation may continue past it.
!
! It differs from interp1d's probe (unit #23) in the one way that matters: there
! the branch assigns a 38-character literal FIRST and the WRITE then overruns
! that record. Here there is no assignment at all, so the WRITE formats into
! `ErrVar%ErrMsg` at whatever LEN it already has -- and `DISCON.F90:159` sets
! `ErrVar%ErrMsg = ''` on every controller call, which is LEN 0.
!
!   gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off \
!       size_mismatch_probe.f90 -o probe && ./probe ; echo "exit $?"
PROGRAM size_mismatch_probe
    IMPLICIT NONE
    CHARACTER(:), ALLOCATABLE :: ErrMsg
    INTEGER :: n_xData, n_zData_2

    ! What DISCON.F90:159 leaves in the field on entry to every controller call.
    ErrMsg = ''
    PRINT *, 'LEN(ErrMsg) as DISCON.F90:159 leaves it =', LEN(ErrMsg)

    n_xData   = 3
    n_zData_2 = 4
    PRINT *, 'characters the WRITE formats =', &
        LEN(' SIZE(xData) =') + 4 + LEN(' and SIZE(zData,1) =') + 4 + &
        LEN(' are not the same')

    ! Functions.f90 (clean) 198-203, verbatim but for the field name.
    WRITE(ErrMsg,"(A,I4,A,I4,A)") " SIZE(xData) =", n_xData, &
    ' and SIZE(zData,1) =', n_zData_2,' are not the same'

    PRINT *, 'the WRITE returned; ErrMsg = ', ErrMsg
END PROGRAM size_mismatch_probe
