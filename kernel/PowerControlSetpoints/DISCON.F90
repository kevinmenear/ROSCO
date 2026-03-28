!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 07:59:15 
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
! High level run script
!=======================================================================


SUBROUTINE discon(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath) 
! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

    USE controllers 
    USE extcontrol 
    USE kgen_utils_mod
    USE kgen_utils_mod

    IMPLICIT NONE 
! Enable .dll export
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------
! Passed Variables:
!REAL(ReKi), INTENT(IN)      :: from_SC(*)       ! DATA from the super controller
!REAL(ReKi), INTENT(INOUT)   :: to_SC(*)         ! DATA to the super controller


    TYPE(controlparameters) :: cntrpar 
    TYPE(localvariables) :: localvar 
    TYPE(objectinstances) :: objinst 
    TYPE(debugvariables) :: debugvar 
    TYPE(errorvariables) :: errvar 


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
! VIT: removed invalid PUBLIC statement
    TYPE(check_t) :: check_status 
    INTEGER*8 :: kgen_start_clock, kgen_stop_clock, kgen_rate_clock 
    REAL(KIND=kgen_dp) :: gkgen_measure 
    TYPE(controlparameters) :: kgenref_cntrpar 
    TYPE(localvariables) :: kgenref_localvar 
    TYPE(objectinstances) :: kgenref_objinst 
    TYPE(debugvariables) :: kgenref_debugvar 
    TYPE(errorvariables) :: kgenref_errvar 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    CALL kr_rosco_types_controlparameters(cntrpar, kgen_unit, "cntrpar", .FALSE.) 
    CALL kr_rosco_types_localvariables(localvar, kgen_unit, "localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(objinst, kgen_unit, "objinst", .FALSE.) 
    CALL kr_rosco_types_debugvariables(debugvar, kgen_unit, "debugvar", .FALSE.) 
    CALL kr_rosco_types_errorvariables(errvar, kgen_unit, "errvar", .FALSE.) 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_controlparameters(kgenref_cntrpar, kgen_unit, "kgenref_cntrpar", .FALSE.) 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(kgenref_objinst, kgen_unit, "kgenref_objinst", .FALSE.) 
    CALL kr_rosco_types_debugvariables(kgenref_debugvar, kgen_unit, "kgenref_debugvar", .FALSE.) 
    CALL kr_rosco_types_errorvariables(kgenref_errvar, kgen_unit, "kgenref_errvar", .FALSE.) 

!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------
! Check for restart


! Read avrSWAP array into derived types/variables

! Set Control Parameters


! Call external controller, if desired


! Filter signals


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
    CALL PowerControlSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)  ! Everything before the pitch, torque set points are computeed
    IF (kgen_mainstage) THEN 
          
        !verify init 
        CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
        CALL kgen_init_check(check_status, rank=kgen_mpirank) 
          
        !extern verify variables 
          
        !local verify variables 
        CALL kv_rosco_types_controlparameters("cntrpar", check_status, cntrpar, kgenref_cntrpar) 
        CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
        CALL kv_rosco_types_objectinstances("objinst", check_status, objinst, kgenref_objinst) 
        CALL kv_rosco_types_debugvariables("debugvar", check_status, debugvar, kgenref_debugvar) 
        CALL kv_rosco_types_errorvariables("errvar", check_status, errvar, kgenref_errvar) 
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
    CALL PowerControlSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)  ! Everything before the pitch, torque set points are computeed
        END DO   
        CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
        kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
        IF (check_status%rank==0) THEN 
            WRITE (*, *) "PowerControlSetpoints : Time per call (usec): ", kgen_measure 
        END IF   
    END IF   
    IF (kgen_warmupstage) THEN 
    END IF   
    IF (kgen_evalstage) THEN 
    END IF   


! Add RoutineName to error message


END SUBROUTINE discon 