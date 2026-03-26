!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:50:11 
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
! This module contains the primary controller routines


MODULE Controllers

    USE controllerblocks 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 

    USE ISO_C_BINDING
    IMPLICIT NONE 


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

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------  
    SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL variablespeedcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(ErrVar))
    END SUBROUTINE VariableSpeedControl
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst)
        USE rosco_types, ONLY: piparams 
    ! PI controller, with output saturation
        USE rosco_types, ONLY: kr_rosco_types_piparams 
        USE rosco_types, ONLY: kv_rosco_types_piparams 


        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)      :: inst
        REAL(DbKi),    INTENT(IN)         :: I0
        TYPE(piParams), INTENT(INOUT)  :: piP
        LOGICAL,    INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term
        ! Initialize persistent variables/arrays, and set inital condition for integrator term

        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            
            PIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIController
!-------------------------------------------------------------------------------------------------------------------------------


    REAL(DbKi) FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar)
        USE rosco_types, ONLY: piparams, localvariables, objectinstances 
    ! PI controller, with output saturation
        USE rosco_types, ONLY: kr_rosco_types_piparams 
        USE rosco_types, ONLY: kr_rosco_types_objectinstances 
        USE rosco_types, ONLY: kr_rosco_types_localvariables 
        USE rosco_types, ONLY: kv_rosco_types_piparams 
        USE rosco_types, ONLY: kv_rosco_types_objectinstances 
        USE rosco_types, ONLY: kv_rosco_types_localvariables 


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


        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers