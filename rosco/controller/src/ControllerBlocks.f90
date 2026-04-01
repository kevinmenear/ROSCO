! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains additional routines and functions to supplement the primary controllers used in the Controllers module

MODULE ControllerBlocks

USE, INTRINSIC :: ISO_C_Binding
USE Constants
USE Filters
USE Functions
USE SysSubs

IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of PitchSaturation
    INTERFACE
        FUNCTION pitchsaturation_c(LocalVar, CntrPar, objInst, DebugVar, ErrVar) BIND(C, NAME='pitchsaturation_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: pitchsaturation_c
        END FUNCTION pitchsaturation_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of StateMachine
    INTERFACE
        SUBROUTINE statemachine_c(CntrPar, LocalVar) BIND(C, NAME='statemachine_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
        END SUBROUTINE statemachine_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of SetpointSmoother
    INTERFACE
        SUBROUTINE setpointsmoother_c(LocalVar, CntrPar, objInst) BIND(C, NAME='setpointsmoother_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE setpointsmoother_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PowerControlSetpoints
    INTERFACE
        SUBROUTINE powercontrolsetpoints_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='powercontrolsetpoints_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE powercontrolsetpoints_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of RefSpeedExclusion
    INTERFACE
        SUBROUTINE refspeedexclusion_c(LocalVar, CntrPar, objInst, DebugVar) BIND(C, NAME='refspeedexclusion_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
        END SUBROUTINE refspeedexclusion_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ComputeVariablesSetpoints
    INTERFACE
        SUBROUTINE computevariablessetpoints_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='computevariablessetpoints_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE computevariablessetpoints_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Shutdown
    INTERFACE
        SUBROUTINE shutdown_c(LocalVar, CntrPar, objInst, ErrVar) BIND(C, NAME='shutdown_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE shutdown_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Startup
    INTERFACE
        SUBROUTINE startup_c(LocalVar, CntrPar, objInst, ErrVar) BIND(C, NAME='startup_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE startup_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of WindSpeedEstimator
    INTERFACE
        SUBROUTINE windspeedestimator_c(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar) BIND(C, NAME='windspeedestimator_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: PerfData
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE windspeedestimator_c
    END INTERFACE

CONTAINS

    SUBROUTINE PowerControlSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
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
        CALL powercontrolsetpoints_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE PowerControlSetpoints

! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
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
        CALL computevariablessetpoints_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE ComputeVariablesSetpoints
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StateMachine(CntrPar, LocalVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL statemachine_c(C_LOC(CntrPar_view), C_LOC(LocalVar))
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        USE vit_performancedata_view, ONLY: performancedata_view_t, vit_populate_performancedata
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(PerformanceData), INTENT(INOUT), TARGET :: PerfData
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(performancedata_view_t), TARGET :: PerfData_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_performancedata(PerfData, PerfData_view)
        CALL windspeedestimator_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(PerfData_view), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL setpointsmoother_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst))
    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar) RESULT(PitchSaturation_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        REAL(DbKi) :: PitchSaturation_result
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        PitchSaturation_result = REAL(pitchsaturation_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar)), DbKi)
    END FUNCTION PitchSaturation
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Startup(LocalVar, CntrPar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL startup_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar))
    END SUBROUTINE Startup
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Shutdown(LocalVar, CntrPar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL shutdown_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar))
    END SUBROUTINE Shutdown
!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL refspeedexclusion_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(DebugVar))
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks
