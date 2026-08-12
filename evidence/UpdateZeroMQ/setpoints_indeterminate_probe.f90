! Does the reference's UNINITIALISED `setpoints(8)` reach the eight LocalVar
! outputs, and is it stable?  ZMQ_CLIENT is NOT defined on this campaign's
! build (rosco/controller/build/CMakeCache.txt: PC_ZeroMQ_FOUND is empty), so
! the `#else` branch is what compiles and NOTHING ever writes `setpoints`.
!
! Three calls: one on a fresh frame, one after a routine that fills the same
! stack region with a recognisable pattern, one after a routine that fills it
! with a different one.
PROGRAM setpoints_probe
    USE ROSCO_Types
    USE ZeroMQInterface
    IMPLICIT NONE
    TYPE(LocalVariables)    :: LocalVar
    TYPE(ControlParameters) :: CntrPar
    TYPE(ErrorVariables)    :: ErrVar

    CntrPar%n_DT_ZMQ      = 1
    CntrPar%ZMQ_Mode      = 1
    CntrPar%ZMQ_CommAddress = 'tcp://localhost:5555'
    LocalVar%n_DT         = 0
    LocalVar%iStatus      = 0

    CALL show('fresh frame     ')
    CALL dirty(1.0D0)
    CALL show('after +1.0 fill ')
    CALL dirty(-7.25D0)
    CALL show('after -7.25 fill')
CONTAINS
    SUBROUTINE dirty(v)
        REAL(8), INTENT(IN) :: v
        REAL(8) :: junk(256)
        INTEGER :: i
        DO i = 1, 256
            junk(i) = v
        END DO
        IF (SUM(junk) == 1.0D99) PRINT *, junk(1)   ! keep it alive
    END SUBROUTINE dirty

    SUBROUTINE show(tag)
        CHARACTER(*), INTENT(IN) :: tag
        CALL UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
        WRITE (*,'(A,A,8(1X,ES23.16))') tag, ' :', &
            LocalVar%ZMQ_TorqueOffset, LocalVar%ZMQ_YawOffset, &
            LocalVar%ZMQ_PitOffset(1), LocalVar%ZMQ_PitOffset(2), &
            LocalVar%ZMQ_PitOffset(3), LocalVar%ZMQ_R_Speed, &
            LocalVar%ZMQ_R_Torque, LocalVar%ZMQ_R_Pitch
        WRITE (*,'(A,I0,A,A)') '   aviFAIL=', ErrVar%aviFAIL, '  ErrMsg=', TRIM(ErrVar%ErrMsg)
    END SUBROUTINE show
END PROGRAM setpoints_probe
