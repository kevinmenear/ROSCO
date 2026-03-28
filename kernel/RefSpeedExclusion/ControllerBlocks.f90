!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 08:52:35 
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
! VIT: removed invalid PUBLIC statement

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

CONTAINS


! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    

SUBROUTINE computevariablessetpoints(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst, &
&debugvar) 
    USE rosco_types, ONLY: controlparameters, localvariables, objectinstances, debugvariables 
        ! Allocate variables
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kr_rosco_types_controlparameters 
    USE rosco_types, ONLY: kr_rosco_types_objectinstances 
    USE rosco_types, ONLY: kr_rosco_types_debugvariables 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_controlparameters 
    USE rosco_types, ONLY: kv_rosco_types_objectinstances 
    USE rosco_types, ONLY: kv_rosco_types_debugvariables 
    TYPE(controlparameters), INTENT(INOUT) :: cntrpar 
    TYPE(localvariables), INTENT(INOUT) :: localvar 
    TYPE(objectinstances), INTENT(INOUT) :: objinst 
    TYPE(debugvariables), INTENT(INOUT) :: debugvar 
        !   Change pitch reference speed
    INTEGER, INTENT(IN) :: kgen_unit 
    REAL(KIND=kgen_dp), INTENT(OUT) :: kgen_measure 
    LOGICAL, INTENT(OUT) :: kgen_isverified 
    CHARACTER(LEN=*), INTENT(IN) :: kgen_filepath 
    LOGICAL :: kgen_istrue 
    REAL(KIND=8) :: kgen_array_sum 
    INTEGER :: kgen_intvar, kgen_ierr 
    INTEGER :: kgen_mpirank, kgen_openmptid, kgen_kernelinvoke 
    LOGICAL :: kgen_evalstage, kgen_warmupstage, kgen_mainstage 
    COMMON / state / kgen_mpirank, kgen_openmptid, kgen_kernelinvoke, kgen_evalstage, kgen_warmupstage, kgen_mainstage 
    INTEGER, PARAMETER :: KGEN_MAXITER = 1 
      
    TYPE(check_t) :: check_status 
    INTEGER*8 :: kgen_start_clock, kgen_stop_clock, kgen_rate_clock 
    REAL(KIND=kgen_dp) :: gkgen_measure 
    TYPE(localvariables) :: kgenref_localvar 
    TYPE(objectinstances) :: kgenref_objinst 
    TYPE(debugvariables) :: kgenref_debugvar 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(kgenref_objinst, kgen_unit, "kgenref_objinst", .FALSE.) 
    CALL kr_rosco_types_debugvariables(kgenref_debugvar, kgen_unit, "kgenref_debugvar", .FALSE.) 


        ! Lookup table for speed setpoint (PRC_Mode 1)
        

        ! Implement setpoint smoothing


        ! Compute error for pitch controller

        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]


        ! Region 3 FBP reference logic, triggers if Region-2 reference speed is higher than rated
        ! DBS: Alternatively, each of these alternative reference modes could identify Region 3 using their reference-deriving signal, e.g. if WE_Vw > rated speed (accessible in ROSCO?) or GenTq > VS_RtTq


        ! Change VS Ref speed based on R_Speed

        ! Filter reference signal


        ! Exclude reference speeds specified by user


    IF (kgen_evalstage) THEN 
    END IF   
    IF (kgen_warmupstage) THEN 
    END IF   
    IF (kgen_mainstage) THEN 
    END IF   
      
    !Uncomment following call statement to turn on perturbation experiment. 
    !Adjust perturbation value and/or kind parameter if required. 
    !CALL kgen_perturb_real( your_variable, 1.0E-15_8 ) 
      
      
    !call to kgen kernel 
            CALL RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
            IF (kgen_mainstage) THEN 
                  
                !verify init 
                CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
                CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                  
                !extern verify variables 
                  
                !local verify variables 
                CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
                CALL kv_rosco_types_objectinstances("objinst", check_status, objinst, kgenref_objinst) 
                CALL kv_rosco_types_debugvariables("debugvar", check_status, debugvar, kgenref_debugvar) 
                IF (check_status%rank == 0) THEN 
                    WRITE (*, *) "" 
                END IF   
                IF (kgen_verboseLevel > 0) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "Number of output variables: ", check_status%numTotal 
                        WRITE (*, *) "Number of identical variables: ", check_status%numIdentical 
                        WRITE (*, *) "Number of non-identical variables within tolerance: ", check_status%numInTol 
                        WRITE (*, *) "Number of non-identical variables out of tolerance: ", check_status%numOutTol 
                        WRITE (*, *) "Tolerance: ", kgen_tolerance 
                    END IF   
                END IF   
                IF (check_status%rank == 0) THEN 
                    WRITE (*, *) "" 
                END IF   
                IF (check_status%numOutTol > 0) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "Verification FAILED with" // TRIM(ADJUSTL(kgen_filepath)) 
                    END IF   
                    check_status%Passed = .FALSE. 
                    kgen_isverified = .FALSE. 
                ELSE 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "Verification PASSED with " // TRIM(ADJUSTL(kgen_filepath)) 
                    END IF   
                    check_status%Passed = .TRUE. 
                    kgen_isverified = .TRUE. 
                END IF   
                IF (check_status%rank == 0) THEN 
                    WRITE (*, *) "" 
                END IF   
                CALL SYSTEM_CLOCK(kgen_start_clock, kgen_rate_clock) 
                DO kgen_intvar = 1, KGEN_MAXITER 
            CALL RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
                END DO   
                CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                IF (check_status%rank==0) THEN 
                    WRITE (*, *) "RefSpeedExclusion : Time per call (usec): ", kgen_measure 
                END IF   
            END IF   
            IF (kgen_warmupstage) THEN 
            END IF   
            IF (kgen_evalstage) THEN 
            END IF   
        ! Saturate torque reference speed below rated speed if using pitch control in Region 3


        ! Simple lookup table for generator speed (PRC_Mode 1)


        ! Implement setpoint smoothing


        ! Force minimum rotor speed

        ! Compute speed error from reference

        ! Define transition region setpoint errors

        ! Region 3 minimum pitch angle for state machine
        
        ! Debug Vars


END SUBROUTINE computevariablessetpoints 
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL refspeedexclusion_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(DebugVar))
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks