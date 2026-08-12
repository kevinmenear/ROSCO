!KGEN-generated Fortran source file 
  
!Generated at : 2026-08-12 16:45:25 
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
    USE kgen_utils_mod, ONLY: kgen_dp, kgen_array_sumcheck 
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 

    IMPLICIT NONE 
    PUBLIC picontroller 

CONTAINS
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


!-------------------------------------------------------------------------------------------------------------------------------
FUNCTION picontroller(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, minvalue, maxvalue, inst, pip) 
    USE rosco_types, ONLY: piparams 
    ! PI controller, with output saturation
    USE kgen_utils_mod, ONLY: kgen_dp, kgen_array_sumcheck 
    USE kgen_utils_mod, ONLY: kgen_perturb_real 
    USE rosco_types, ONLY: kr_rosco_types_piparams 
    USE rosco_types, ONLY: kv_rosco_types_piparams 


    IMPLICIT NONE 
        ! Allocate Inputs
    REAL(KIND=dbki), INTENT(INOUT) :: minvalue 
    REAL(KIND=dbki), INTENT(INOUT) :: maxvalue 
    INTEGER(KIND=intki), INTENT(INOUT) :: inst 
    TYPE(piparams), INTENT(INOUT) :: pip 
        ! Allocate local variables
    REAL(KIND=dbki) :: pterm 
        ! Initialize persistent variables/arrays, and set inital condition for integrator term
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
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    READ (UNIT = kgen_unit) pterm 
      
    !extern output variables 
      
    !local output variables 

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
            PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
            IF (kgen_mainstage) THEN 
                  
                !verify init 
                CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=1) 
                CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                  
                !extern verify variables 
                  
                !local verify variables 
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
            PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
                END DO   
                CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                IF (check_status%rank==0) THEN 
                    WRITE (*, *) "saturate : Time per call (usec): ", kgen_measure 
                END IF   
            END IF   
            IF (kgen_warmupstage) THEN 
            END IF   
            IF (kgen_evalstage) THEN 
            END IF   
        
END SUBROUTINE picontroller 
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers