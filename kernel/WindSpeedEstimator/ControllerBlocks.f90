!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 09:17:54 
!KGEN version : 0.8.1 
  
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

    USE filters 
    USE syssubs 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 

    USE ISO_C_BINDING
    IMPLICIT NONE 


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


! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        USE vit_performancedata_view, ONLY: performancedata_view_t, vit_populate_performancedata, vit_original_performancedata
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
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        vit_original_performancedata => PerfData
        CALL windspeedestimator_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(PerfData_view), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks