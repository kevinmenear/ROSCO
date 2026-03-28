!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 07:59:49 
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

CONTAINS


! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    

    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL computevariablessetpoints_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE ComputeVariablesSetpoints
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar) 
    ! Reference speed exclusion:
    !   Changes torque controllerr reference speed to avoid specified frequencies by a prescribed bandwidth
        USE rosco_types, ONLY: localvariables, controlparameters, debugvariables, objectinstances 
        USE rosco_types, ONLY: kr_rosco_types_localvariables 
        USE rosco_types, ONLY: kr_rosco_types_controlparameters 
        USE rosco_types, ONLY: kr_rosco_types_objectinstances 
        USE rosco_types, ONLY: kr_rosco_types_debugvariables 
        USE rosco_types, ONLY: kv_rosco_types_localvariables 
        USE rosco_types, ONLY: kv_rosco_types_controlparameters 
        USE rosco_types, ONLY: kv_rosco_types_objectinstances 
        USE rosco_types, ONLY: kv_rosco_types_debugvariables 
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(DebugVariables),      INTENT(INOUT)        :: DebugVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst

        
        REAL(DbKi)                             :: VS_RefSpeed_LSS
        ! Get LSS Ref speed
        
        VS_RefSpeed_LSS = LocalVar%VS_RefSpd/CntrPar%WE_GearboxRatio

        IF ((VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2) .AND. &
            (VS_RefSpeed_LSS < CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2)) THEN
            ! In hysteresis zone, hold reference speed
            LocalVar%FA_Hist = 1 ! Set negative hysteris if ref < exclusion band
        ELSE
            LocalVar%FA_Hist = 0
        END IF
        ! Initialize last reference speed state

        IF (LocalVar%restart /= 0) THEN
            ! If starting in hist band
            IF (LocalVar%FA_Hist > 0) THEN
                IF (VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed) THEN
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2
                ELSE
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2
                ENDIF
            ELSE
                LocalVar%TRA_LastRefSpd = VS_RefSpeed_LSS
            END IF
        END IF 


        IF (LocalVar%FA_Hist > 0) THEN
            LocalVar%VS_RefSpd_TRA = LocalVar%TRA_LastRefSpd
        ELSE
            LocalVar%VS_RefSpd_TRA = VS_RefSpeed_LSS
        END IF
        ! Save last reference speed       

        LocalVar%TRA_LastRefSpd = LocalVar%VS_RefSpd_TRA
        ! Rate limit reference speed

        LocalVar%VS_RefSpd_RL = ratelimit(LocalVar%VS_RefSpd_TRA, -CntrPar%TRA_RateLimit, CntrPar%TRA_RateLimit, LocalVar%DT, (LocalVar%restart /= 0), LocalVar%rlP,objInst%instRL)
        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_RL * CntrPar%WE_GearboxRatio


        
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks