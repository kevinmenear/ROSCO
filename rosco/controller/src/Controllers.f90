! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains the primary controller routines

MODULE Controllers

    USE, INTRINSIC :: ISO_C_Binding
    USE Functions
    USE Filters
    USE ControllerBlocks

    IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of PIController
    INTERFACE
        FUNCTION picontroller_c(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst) BIND(C, NAME='picontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: I0
            TYPE(C_PTR), VALUE :: piP
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: picontroller_c
        END FUNCTION picontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PIIController
    INTERFACE
        FUNCTION piicontroller_c(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) BIND(C, NAME='piicontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: error2
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: ki2
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: I0
            TYPE(C_PTR), VALUE :: piP
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: piicontroller_c
        END FUNCTION piicontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ResController
    INTERFACE
        FUNCTION rescontroller_c(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) BIND(C, NAME='rescontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: freq
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            TYPE(C_PTR), VALUE :: resP
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: rescontroller_c
        END FUNCTION rescontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ForeAftDamping
    INTERFACE
        SUBROUTINE foreaftdamping_c(CntrPar, LocalVar, objInst) BIND(C, NAME='foreaftdamping_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE foreaftdamping_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of FloatingFeedback
    INTERFACE
        FUNCTION floatingfeedback_c(LocalVar, CntrPar, objInst, ErrVar) BIND(C, NAME='floatingfeedback_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: floatingfeedback_c
        END FUNCTION floatingfeedback_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of StructuralControl
    INTERFACE
        SUBROUTINE structuralcontrol_c(avrSWAP, CntrPar, LocalVar, objInst, ErrVar) BIND(C, NAME='structuralcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE structuralcontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of CableControl
    INTERFACE
        SUBROUTINE cablecontrol_c(avrSWAP, CntrPar, LocalVar, objInst, ErrVar) BIND(C, NAME='cablecontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE cablecontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of FlapControl
    INTERFACE
        SUBROUTINE flapcontrol_c(avrSWAP, CntrPar, LocalVar, objInst) BIND(C, NAME='flapcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE flapcontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of YawRateControl
    INTERFACE
        SUBROUTINE yawratecontrol_c(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='yawratecontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE yawratecontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of VariableSpeedControl
    INTERFACE
        SUBROUTINE variablespeedcontrol_c(avrSWAP, CntrPar, LocalVar, objInst, ErrVar) BIND(C, NAME='variablespeedcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE variablespeedcontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of IPC
    INTERFACE
        SUBROUTINE ipc_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='ipc_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE ipc_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ActiveWakeControl
    INTERFACE
        SUBROUTINE activewakecontrol_c(CntrPar, LocalVar, DebugVar, objInst) BIND(C, NAME='activewakecontrol_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE activewakecontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PitchControl
    INTERFACE
        SUBROUTINE pitchcontrol_c(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='pitchcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE pitchcontrol_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL pitchcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE PitchControl
!-------------------------------------------------------------------------------------------------------------------------------  
    SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL variablespeedcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(ErrVar))
    END SUBROUTINE VariableSpeedControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL yawratecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE YawRateControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL ipc_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE IPC
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ForeAftDamping(CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL foreaftdamping_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst))
    END SUBROUTINE ForeAftDamping
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar) RESULT(FloatingFeedback_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        REAL(DbKi) :: FloatingFeedback_result
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        FloatingFeedback_result = REAL(floatingfeedback_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar)), DbKi)
    END FUNCTION FloatingFeedback
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL flapcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst))
    END SUBROUTINE FlapControl


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL activewakecontrol_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(DebugVar), C_LOC(objInst))
    END SUBROUTINE ActiveWakeControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE CableControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL cablecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(ErrVar))
    END SUBROUTINE CableControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StructuralControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL structuralcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(ErrVar))
    END SUBROUTINE StructuralControl
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: error
        REAL(DbKi), INTENT(IN) :: kp
        REAL(DbKi), INTENT(IN) :: ki
        REAL(DbKi), INTENT(IN) :: minValue
        REAL(DbKi), INTENT(IN) :: maxValue
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: I0
        TYPE(piParams), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi) :: PIController_result
        PIController_result = REAL(picontroller_c(REAL(error, C_DOUBLE), REAL(kp, C_DOUBLE), REAL(ki, C_DOUBLE), REAL(minValue, C_DOUBLE), REAL(maxValue, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(I0, C_DOUBLE), C_LOC(piP), LOGICAL(reset, C_BOOL), inst), DbKi)
    END FUNCTION PIController


!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar)
        USE ROSCO_Types, ONLY : piParams, LocalVariables, ObjectInstances

    ! PI controller, with output saturation

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: kd
        REAL(DbKi),    INTENT(IN)         :: tf
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        TYPE(ObjectInstances),      INTENT(INOUT)   :: objInst  ! all object instances (PI, filters used here)
        TYPE(LocalVariables),       INTENT(INOUT)   :: LocalVar

        REAL(DbKi),    INTENT(IN)           :: I0
        TYPE(piParams), INTENT(INOUT)       :: piP
        LOGICAL,    INTENT(IN)              :: reset     
        
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                          :: PTerm, DTerm                                 ! Proportional, deriv. terms
        REAL(DbKi)                          :: EFilt                    ! Filtered error for derivative

        ! Always filter error
        EFilt = LPFilter(error, DT, tf, LocalVar%FP, LocalVar%iStatus, reset, objInst%instLPF)

        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF (reset) THEN
            piP%ITerm(objInst%instPI) = I0
            piP%ITermLast(objInst%instPI) = I0
            piP%ELast(objInst%instPI) = 0.0_DbKi
            PIDController = I0
        ELSE
            ! Proportional
            PTerm = kp*error
            
            ! Integrate and saturate
            piP%ITerm(objInst%instPI) = piP%ITerm(objInst%instPI) + DT*ki*error
            piP%ITerm(objInst%instPI) = saturate(piP%ITerm(objInst%instPI), minValue, maxValue)

            ! Derivative (filtered)
            DTerm = kd * (EFilt - piP%ELast(objInst%instPI)) / DT
            
            ! Saturate all
            PIDController = saturate(PTerm + piP%ITerm(objInst%instPI) + DTerm, minValue, maxValue)
        
            ! Save lasts
            piP%ITermLast(objInst%instPI) = piP%ITerm(objInst%instPI)
            piP%ELast(objInst%instPI) = EFilt
        END IF
        objInst%instPI = objInst%instPI + 1
        
    END FUNCTION PIDController

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: error
        REAL(DbKi), INTENT(IN) :: error2
        REAL(DbKi), INTENT(IN) :: kp
        REAL(DbKi), INTENT(IN) :: ki
        REAL(DbKi), INTENT(IN) :: ki2
        REAL(DbKi), INTENT(IN) :: minValue
        REAL(DbKi), INTENT(IN) :: maxValue
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: I0
        TYPE(piParams), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi) :: PIIController_result
        PIIController_result = REAL(piicontroller_c(REAL(error, C_DOUBLE), REAL(error2, C_DOUBLE), REAL(kp, C_DOUBLE), REAL(ki, C_DOUBLE), REAL(ki2, C_DOUBLE), REAL(minValue, C_DOUBLE), REAL(maxValue, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(I0, C_DOUBLE), C_LOC(piP), LOGICAL(reset, C_BOOL), inst), DbKi)
    END FUNCTION PIIController

        !-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) RESULT(ResController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : resParams
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: error
        REAL(DbKi), INTENT(IN) :: kp
        REAL(DbKi), INTENT(IN) :: ki
        REAL(DbKi), INTENT(IN) :: freq
        REAL(DbKi), INTENT(IN) :: minValue
        REAL(DbKi), INTENT(IN) :: maxValue
        REAL(DbKi), INTENT(IN) :: DT
        TYPE(resParams), INTENT(INOUT), TARGET :: resP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi) :: ResController_result
        ResController_result = REAL(rescontroller_c(REAL(error, C_DOUBLE), REAL(kp, C_DOUBLE), REAL(ki, C_DOUBLE), REAL(freq, C_DOUBLE), REAL(minValue, C_DOUBLE), REAL(maxValue, C_DOUBLE), REAL(DT, C_DOUBLE), C_LOC(resP), LOGICAL(reset, C_BOOL), inst), DbKi)
    END FUNCTION ResController

!-------------------------------------------------------------------------------------------------------------------------------
END MODULE Controllers
