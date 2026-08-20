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
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: picontroller_c
        END FUNCTION picontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PIDController
    INTERFACE
        FUNCTION pidcontroller_c(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar) BIND(C, NAME='pidcontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: kd
            REAL(C_DOUBLE), VALUE :: tf
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: I0
            TYPE(C_PTR), VALUE :: piP
            INTEGER(C_INT), VALUE :: reset
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: LocalVar
            REAL(C_DOUBLE) :: pidcontroller_c
        END FUNCTION pidcontroller_c
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
            INTEGER(C_INT), VALUE :: reset
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
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: rescontroller_c
        END FUNCTION rescontroller_c
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


    ! Auto-generated interface for C++ implementation of ForeAftDamping
    INTERFACE
        SUBROUTINE foreaftdamping_c(CntrPar, LocalVar, objInst) BIND(C, NAME='foreaftdamping_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE foreaftdamping_c
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
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL pitchcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE PitchControl
!-------------------------------------------------------------------------------------------------------------------------------  
    SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL variablespeedcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE VariableSpeedControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL yawratecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE YawRateControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL ipc_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE IPC
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ForeAftDamping(CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL foreaftdamping_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE ForeAftDamping
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar) RESULT(FloatingFeedback_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: FloatingFeedback_result
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        FloatingFeedback_result = REAL(floatingfeedback_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION FloatingFeedback
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL flapcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE FlapControl


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL activewakecontrol_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(DebugVar), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE ActiveWakeControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE CableControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL cablecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE CableControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StructuralControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL structuralcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE StructuralControl
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: I0
        TYPE(PIPARAMS), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8) :: PIController_result
        PIController_result = REAL(picontroller_c(error, kp, ki, minValue, maxValue, DT, I0, C_LOC(piP), MERGE(1_C_INT, 0_C_INT, reset), inst), 8)
    END FUNCTION PIController


!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar) RESULT(PIDController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams, LocalVariables, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: kd
        REAL(8), INTENT(IN) :: tf
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: I0
        TYPE(PIPARAMS), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        REAL(8) :: PIDController_result
        TYPE(C_PTR) :: piP_cptr
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        ! Resolve aliasing: an argument that IS a component of a view-type
        ! argument must be written through the VIEW, or the view copy-back
        ! afterwards would discard those writes (see _aliased_component_args).
        IF (C_ASSOCIATED(C_LOC(piP), C_LOC(LocalVar%piP))) THEN
            piP_cptr = C_LOC(LocalVar_view%piP)
        ELSE
            piP_cptr = C_LOC(piP)
        END IF
        PIDController_result = REAL(pidcontroller_c(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP_cptr, MERGE(1_C_INT, 0_C_INT, reset), C_LOC(objInst), C_LOC(LocalVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END FUNCTION PIDController

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: error2
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: ki2
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: I0
        TYPE(PIPARAMS), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8) :: PIIController_result
        PIIController_result = REAL(piicontroller_c(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, C_LOC(piP), MERGE(1_C_INT, 0_C_INT, reset), inst), 8)
    END FUNCTION PIIController

        !-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) RESULT(ResController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : resParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: freq
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        TYPE(RESPARAMS), INTENT(INOUT), TARGET :: resP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8) :: ResController_result
        ResController_result = REAL(rescontroller_c(error, kp, ki, freq, minValue, maxValue, DT, C_LOC(resP), MERGE(1_C_INT, 0_C_INT, reset), inst), 8)
    END FUNCTION ResController

!-------------------------------------------------------------------------------------------------------------------------------
END MODULE Controllers
