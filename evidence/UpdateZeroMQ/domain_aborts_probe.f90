! Two shapes on which the REFERENCE stops rather than answers.  Each is
! selected by argv(1) so a crash in one cannot hide the other.
!
!   mod0     CntrPar%n_DT_ZMQ = 0  ->  MOD(LocalVar%n_DT, 0)
!   addr256  CntrPar%ZMQ_CommAddress full width -> the internal WRITE needs
!            LEN_TRIM+1 = 257 characters of a CHARACTER(256) record
PROGRAM domain_aborts
    USE ROSCO_Types
    USE ZeroMQInterface
    IMPLICIT NONE
    TYPE(LocalVariables)    :: LocalVar
    TYPE(ControlParameters) :: CntrPar
    TYPE(ErrorVariables)    :: ErrVar
    CHARACTER(32) :: which

    CALL GET_COMMAND_ARGUMENT(1, which)
    CntrPar%n_DT_ZMQ = 1
    CntrPar%ZMQ_Mode = 1
    CntrPar%ZMQ_CommAddress = 'tcp://localhost:5555'
    LocalVar%n_DT    = 5
    LocalVar%iStatus = 0

    IF (TRIM(which) == 'mod0')    CntrPar%n_DT_ZMQ = 0
    IF (TRIM(which) == 'addr256') CntrPar%ZMQ_CommAddress = REPEAT('a', 256)

    PRINT *, 'calling with ', TRIM(which)
    CALL UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
    PRINT *, 'RETURNED NORMALLY, aviFAIL=', ErrVar%aviFAIL
END PROGRAM domain_aborts
