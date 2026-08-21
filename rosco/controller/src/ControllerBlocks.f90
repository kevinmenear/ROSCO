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


    ! Auto-generated interface for C++ implementation of StateMachine
    INTERFACE
        SUBROUTINE statemachine_c(CntrPar, LocalVar) BIND(C, NAME='statemachine_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
        END SUBROUTINE statemachine_c
    END INTERFACE


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


    ! Auto-generated interface for C++ implementation of SetpointSmoother
    INTERFACE
        SUBROUTINE setpointsmoother_c(LocalVar, CntrPar, objInst) BIND(C, NAME='setpointsmoother_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE setpointsmoother_c
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
        CALL powercontrolsetpoints_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE PowerControlSetpoints

! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
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
        CALL computevariablessetpoints_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ComputeVariablesSetpoints
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StateMachine(CntrPar, LocalVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL statemachine_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, DebugVariables, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_performancedata_view, ONLY: performancedata_view_t, vit_populate_performancedata, vit_copy_scalars_to_performancedata
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(PERFORMANCEDATA), INTENT(INOUT), TARGET :: PerfData
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(performancedata_view_t), TARGET :: PerfData_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_performancedata(PerfData, PerfData_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL windspeedestimator_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(PerfData_view), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_performancedata(PerfData_view, PerfData)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL setpointsmoother_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar) RESULT(PitchSaturation_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: PitchSaturation_result
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        PitchSaturation_result = REAL(pitchsaturation_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION PitchSaturation
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Startup(LocalVar, CntrPar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL startup_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE Startup
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Shutdown(LocalVar, CntrPar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL shutdown_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE Shutdown
!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL refspeedexclusion_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(DebugVar))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks
