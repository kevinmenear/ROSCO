!KGEN-generated Fortran source file 
  
!Generated at : 2026-08-11 10:25:53 
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
! Read and set the parameters used by the controller


MODULE ReadSetParameters


    USE functions 
    USE rosco_helpers 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 
    IMPLICIT NONE 
    PUBLIC readcontrolparameterfilesub 


CONTAINS
 ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    


! -----------------------------------------------------------------------------------
    ! Define parameters for control actions


    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    ! Also, all computed CntrPar%* parameters should be computed in this subroutine
    
SUBROUTINE readcontrolparameterfilesub(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, accinfile_size, accinfile) 
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE kgen_utils_mod

    INTEGER(KIND=intki), INTENT(INOUT) :: accinfile_size 
    CHARACTER(LEN=accinfile_size), INTENT(INOUT) :: accinfile(accinfile_size) 

        

    CHARACTER(LEN=1024) :: pripath 


        
        ! Get primary path of DISCON.IN file (accINFILE(1) here)
        !$kgen begin_callsite GetPath
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
    CHARACTER(LEN=1024) :: kgenref_pripath 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    READ (UNIT = kgen_unit) pripath 
      
    !extern output variables 
      
    !local output variables 
    READ (UNIT = kgen_unit) kgenref_pripath 

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
        CALL GetPath( accINFILE(1), PriPath )     ! Input files will be relative to the path where the primary input file is located.
        IF (kgen_mainstage) THEN 
              
            !verify init 
            CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
            CALL kgen_init_check(check_status, rank=kgen_mpirank) 
              
            !extern verify variables 
              
            !local verify variables 
            CALL kv_kgen_readcontrolparameterfilesub_subp0("pripath", check_status, pripath, kgenref_pripath) 
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
        CALL GetPath( accINFILE(1), PriPath )     ! Input files will be relative to the path where the primary input file is located.
            END DO   
            CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
            kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
            IF (check_status%rank==0) THEN 
                WRITE (*, *) "GetPath : Time per call (usec): ", kgen_measure 
            END IF   
        END IF   
        IF (kgen_warmupstage) THEN 
        END IF   
        IF (kgen_evalstage) THEN 
        END IF   
        !$kgen end_callsite
        ! Read all lines, first get the number of lines


        ! Close Input File

        ! Read Echo first, so file can be set up, if desired


        ! Set up echo file


        !----------------------- Simulation Control --------------------------


        !----------------- CONTROLLER FLAGS ---------------------


        !----------------- FILTER CONSTANTS ---------------------

        ! Optional filter inds


        !----------- BLADE PITCH CONTROLLER CONSTANTS -----------


        !------------------- IPC CONSTANTS -----------------------


        !------------ VS TORQUE CONTROL CONSTANTS ----------------


        !------------ Fixed-Pitch Region 3 Control ------------


        !------- Setpoint Smoother --------------------------------


        !------------ POWER REFERENCE TRACKING SETPOINTS --------------

        !------------ WIND SPEED ESTIMATOR CONTANTS --------------


        ! Retired WSE inputs:  not used anywhere in code
        ! CALL ParseInput(FileLines,  'WE_CP_n',accINFILE(1),CntrPar%WE_CP_n,ErrVar, UnEc)
        ! CALL ParseAry(  FileLines,  'WE_CP', CntrPar%WE_CP, CntrPar%WE_CP_n, accINFILE(1), ErrVar, .FALSE. , UnEc)
        !-------------- YAW CONTROLLER CONSTANTS -----------------


        !------------ FORE-AFT TOWER DAMPER CONSTANTS ------------


        !------------ PEAK SHAVING ------------


        !------------ STARTUP ------------
        
        !------------ SHUTDOWN ------------


        !------------ FLOATING ------------


        !------------ Flaps ------------


        !------------ Open loop input ------------
        ! Indices can be left 0 by default, checked later


        !------------ Pitch Actuator Inputs ------------


        !------------ Pitch Actuator Faults ------------


        !------------ AWC input ------------


        !------------ External control interface ------------


        !------------ ZeroMQ ------------


        !------------- Cable Control ----- 


        !------------- StC Control ----- 


        ! Open loop cable, structural control, needs number of groups


        !-------------------
        !------------------- CALCULATED CONSTANTS -----------------------
        !----------------------------------------------------------------
        ! Fix defaults manually for now


        ! DT_Out

        ! Fix Paths (add relative paths if called from another dir, UnEc)


        ! Convert yaw rate to deg/s
        
        

        ! Read open loop input, if desired


        !------------------- HOUSEKEEPING -----------------------

        
        ! Add RoutineName to error message
        


          
        CONTAINS 
          


        !verify state subroutine for kv_kgen_readcontrolparameterfilesub_subp0 
        RECURSIVE SUBROUTINE kv_kgen_readcontrolparameterfilesub_subp0(varname, check_status, var, kgenref_var) 
            CHARACTER(LEN=*), INTENT(IN) :: varname 
            TYPE(check_t), INTENT(INOUT) :: check_status 
            CHARACTER(LEN=1024), INTENT(IN) :: var, kgenref_var 
            INTEGER :: check_result 
            LOGICAL :: is_print = .FALSE. 
              
            character(LEN=1024) :: diff 
              
            check_status%numTotal = check_status%numTotal + 1 
              
            IF ((var == kgenref_var) .OR. ((var /= var) .AND. (kgenref_var /= kgenref_var))) THEN
        IF (var /= var) WRITE(*, *) trim(adjustl(varname))," is IDENTICAL (both NaN, uninitialized)." 
                check_status%numIdentical = check_status%numIdentical + 1 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) trim(adjustl(varname)), " is IDENTICAL." 
                    END IF   
                END IF   
                check_result = CHECK_IDENTICAL 
                WRITE(*, *) "[VIT_FIELD] ", trim(adjustl(varname)), " | IDENTICAL | ", var, " | ", kgenref_var
            ELSE 
                check_status%numOutTol = check_status%numOutTol + 1 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) trim(adjustl(varname)), " is NOT IDENTICAL." 
                    END IF   
                END IF   
                check_result = CHECK_OUT_TOL 
                WRITE(*, *) "[VIT_FIELD] ", trim(adjustl(varname)), " | OUT_TOL | ", var, " | ", kgenref_var, " | ", diff
            END IF   
            IF (check_result == CHECK_IDENTICAL) THEN 
                IF (kgen_verboseLevel > 2) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "NOT IMPLEMENTED" 
                        WRITE (*, *) "" 
                    END IF   
                END IF   
            ELSE IF (check_result == CHECK_OUT_TOL) THEN 
                IF (kgen_verboseLevel > 0) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "NOT IMPLEMENTED" 
                        WRITE (*, *) "" 
                    END IF   
                END IF   
            ELSE IF (check_result == CHECK_IN_TOL) THEN 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "NOT IMPLEMENTED" 
                        WRITE (*, *) "" 
                    END IF   
                END IF   
            END IF   
              
        END SUBROUTINE kv_kgen_readcontrolparameterfilesub_subp0 
          
END SUBROUTINE readcontrolparameterfilesub 
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file


    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution


    
END MODULE ReadSetParameters