! WHAT THE REFERENCE DOES FOR EACH VALUE OF PIDController's `objInst%instLPF`.
!
! Modelled on evidence/FindLine/arylen_probe.f90 (unit #32) -- same shape, same
! reason. `harness/ranges.toml` is the only place a human judgement about an
! admissible domain is recorded, and an entry resting on "that looks wrong" does
! not belong there. So this asks the REFERENCE, one value per process, and the
! answer is its exit status.
!
! WHY THIS PARAMETER AND NOT `instPI`. Both are 1-based Fortran indices into
! DIMENSION(1024) arrays. `instPI` subscripts `piP%ITerm` etc. IN THIS UNIT'S
! OWN BODY, so the harness's index inference reads it off both the reference and
! the translation and R5 sweeps it at 1 / interior / n. `instLPF` is subscripted
! inside the CALLEE:
!
!     EFilt = LPFilter(error, DT, tf, LocalVar%FP, LocalVar%iStatus, reset,
!                      objInst%instLPF)                    <- Controllers.f90:1100
!     FP%lpf1_a1(inst) = 2 + CornerFreq*DT                 <- Filters.f90, LPFilter
!
! and the array it indexes is a field of `LocalVar%FP`, a NESTED type the
! generator does not expand into parameters. So there is no compared
! out-parameter for the inference to attach it to, it stays an ordinary defaulted
! integer, and R6's ladder puts both 32-bit extremes in the corpus.
!
! The whole struct is INTENT(INOUT) and every field is compared, so this probe
! reports the two fields the call advances as well as the return value: a run
! that returns without advancing them would be a different answer from a run
! that returns.
PROGRAM instlpf_probe
    USE ROSCO_Types, ONLY : LocalVariables, ObjectInstances, piParams
    USE Controllers, ONLY : PIDController
    IMPLICIT NONE
    TYPE(LocalVariables)  :: LocalVar
    TYPE(ObjectInstances) :: objInst
    TYPE(piParams)        :: piP
    REAL(8)               :: r
    INTEGER(4)            :: instLPF
    CHARACTER(32)         :: arg

    CALL GET_COMMAND_ARGUMENT(1, arg)
    READ(arg, *) instLPF

    ! Ordinary, in-domain values for everything the probe is not asking about.
    ! `instPI` is 1, which the harness's own corpus now holds inside 1..1024.
    LocalVar%iStatus = 0
    LocalVar%FP%lpf1_a1 = 1.0d0
    LocalVar%FP%lpf1_a0 = 0.0d0
    LocalVar%FP%lpf1_b1 = 1.0d0
    LocalVar%FP%lpf1_b0 = 0.0d0
    LocalVar%FP%lpf1_InputSignalLast = 0.0d0
    LocalVar%FP%lpf1_OutputSignalLast = 0.0d0
    piP%ITerm = 0.0d0
    piP%ITermLast = 0.0d0
    piP%ELast = 0.0d0
    objInst%instPI = 1
    objInst%instLPF = instLPF

    WRITE(*,'(A,I12)', ADVANCE='NO') 'instLPF=', instLPF
    FLUSH(6)
    r = PIDController(0.5d0, 1.0d0, 1.0d0, 1.0d0, 0.1d0, -10.0d0, 10.0d0, &
                      0.0125d0, 0.0d0, piP, .FALSE., objInst, LocalVar)
    WRITE(*,'(A,ES23.16,A,I12,A,I12)') '  RETURNED  result=', r, &
        '  instLPF=', objInst%instLPF, '  instPI=', objInst%instPI
END PROGRAM instlpf_probe
