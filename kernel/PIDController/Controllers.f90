!KGEN-generated Fortran source file 
  
!Generated at : 2026-04-01 18:03:41 
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
! VIT: removed invalid PUBLIC statement

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
            LOGICAL(C_BOOL), VALUE :: reset
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: LocalVar
            REAL(C_DOUBLE) :: pidcontroller_c
        END FUNCTION pidcontroller_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------  
SUBROUTINE variablespeedcontrol(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst) 
    ! Generator torque controller
    !       VS_State = VS_State_Error             (0), Error state, for debugging purposes, GenTq = VS_RtTq
    !       VS_State = VS_State_Region_1_5        (1), Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
    !       VS_State = VS_State_Region_2          (2), Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
    !       VS_State = VS_State_Region_2_5        (3), Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
    !       VS_State = VS_State_Region_3_ConstTrq (4), above-rated operation using pitch control (constant torque mode)
    !       VS_State = VS_State_Region_3_ConstPwr (5), above-rated operation using pitch and torque control (constant power mode)
    !       VS_State = VS_State_PI                (6), Tip-Speed-Ratio tracking PI controller (ignore state machine)
    USE rosco_types, ONLY: controlparameters, localvariables, objectinstances 
        ! Inputs
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kr_rosco_types_controlparameters 
    USE rosco_types, ONLY: kr_rosco_types_objectinstances 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_controlparameters 
    USE rosco_types, ONLY: kv_rosco_types_objectinstances 
    TYPE(controlparameters), INTENT(INOUT) :: cntrpar 
    TYPE(localvariables), INTENT(INOUT) :: localvar 
    TYPE(objectinstances), INTENT(INOUT) :: objinst 

        ! Allocate Variables
        ! -------- Variable-Speed Torque Controller --------
        ! Pre-compute generator torque values for K*Omega^2 and constant power
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
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(kgenref_objinst, kgen_unit, "kgenref_objinst", .FALSE.) 


        ! Determine maximum torque saturation limit, VS_MaxTq


        ! Optimal Tip-Speed-Ratio tracking controller (reference generated in subroutine ComputeVariablesSetpoints)


        ! Shutdown
        
        

        ! Saturate based on most stringent defined maximum

        ! Saturate the commanded torque using the torque rate limit

        ! Open loop torque control


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
                LocalVar%GenTqAz = PIDController(LocalVar%AzError, CntrPar%RP_Gains(1), CntrPar%RP_Gains(2), CntrPar%RP_Gains(3), CntrPar%RP_Gains(4), -LocalVar%VS_MaxTq * 2, LocalVar%VS_MaxTq * 2, LocalVar%DT, 0.0_DbKi, LocalVar%piP, (LocalVar%restart /= 0), objInst, LocalVar)
                IF (kgen_mainstage) THEN 
                      
                    !verify init 
                    CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
                    CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                      
                    !extern verify variables 
                      
                    !local verify variables 
                    CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
                    CALL kv_rosco_types_objectinstances("objinst", check_status, objinst, kgenref_objinst) 
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
                LocalVar%GenTqAz = PIDController(LocalVar%AzError, CntrPar%RP_Gains(1), CntrPar%RP_Gains(2), CntrPar%RP_Gains(3), CntrPar%RP_Gains(4), -LocalVar%VS_MaxTq * 2, LocalVar%VS_MaxTq * 2, LocalVar%DT, 0.0_DbKi, LocalVar%piP, (LocalVar%restart /= 0), objInst, LocalVar)
                    END DO   
                    CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                    kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                    IF (check_status%rank==0) THEN 
                        WRITE (*, *) "PIDController : Time per call (usec): ", kgen_measure 
                    END IF   
                END IF   
                IF (kgen_warmupstage) THEN 
                END IF   
                IF (kgen_evalstage) THEN 
                END IF   
        ! Reset the value of LocalVar%VS_LastGenTrq to the current values:

        ! Set the command generator torque (See Appendix A of Bladed User's Guide):
        
        ! Add RoutineName to error message


END SUBROUTINE variablespeedcontrol 
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


    FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar) RESULT(PIDController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams, LocalVariables, ObjectInstances
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: error
        REAL(DbKi), INTENT(IN) :: kp
        REAL(DbKi), INTENT(IN) :: ki
        REAL(DbKi), INTENT(IN) :: kd
        REAL(DbKi), INTENT(IN) :: tf
        REAL(DbKi), INTENT(IN) :: minValue
        REAL(DbKi), INTENT(IN) :: maxValue
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: I0
        TYPE(piParams), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        REAL(DbKi) :: PIDController_result
        PIDController_result = REAL(pidcontroller_c(REAL(error, C_DOUBLE), REAL(kp, C_DOUBLE), REAL(ki, C_DOUBLE), REAL(kd, C_DOUBLE), REAL(tf, C_DOUBLE), REAL(minValue, C_DOUBLE), REAL(maxValue, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(I0, C_DOUBLE), C_LOC(piP), LOGICAL(reset, C_BOOL), C_LOC(objInst), C_LOC(LocalVar)), DbKi)
    END FUNCTION PIDController
!-------------------------------------------------------------------------------------------------------------------------------


        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers