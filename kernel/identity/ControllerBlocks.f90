!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-18 09:47:04 
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

    IMPLICIT NONE 
    PUBLIC windspeedestimator 

CONTAINS


! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
SUBROUTINE windspeedestimator(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, localvar) 
    ! Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
    !       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
    !       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 
    USE rosco_types, ONLY: localvariables 
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    IMPLICIT NONE 
        ! Inputs
    
    TYPE(localvariables), INTENT(INOUT) :: localvar 
        ! Allocate Variables

        !       Only used in EKF, if WE_Mode = 2

        ! REAL(DbKi), DIMENSION(3,3) :: I
        !           - operating conditions
        !           - Covariance matrices

    REAL(KIND=dbki), dimension(1,3) :: h 

        
        ! Saturate inputs to WSE:
        ! Rotor speed
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
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    READ (UNIT = kgen_unit) kgen_istrue 
    IF (kgen_istrue) THEN 
        READ (UNIT = kgen_unit) kgen_array_sum 
        READ (UNIT = kgen_unit) h 
        CALL kgen_array_sumcheck("h", kgen_array_sum, DBLE(SUM(h, mask=(h .eq. h))), .TRUE.) 
    END IF   
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 

        


        ! Blade pitch

            

        
        ! Gen torque


        ! Check to see if in operational range


        ! Restart flag for WSE


        ! Filter the wind speed at hub height regardless, only use if WE_Mode = 0 or WE_Op = 0
        ! Re-initialize at WE_Vw if leaving operational wind, WE_Vw is initialized at HorWindV

        ! ---- Debug Inputs ------

        ! ---- Define wind speed estimate ---- 
        ! Inversion and Invariance Filter implementation

        
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
                LocalVar%WE%P = MATMUL(identity(3) - MATMUL(LocalVar%WE%K,H),LocalVar%WE%P)
                IF (kgen_mainstage) THEN 
                      
                    !verify init 
                    CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
                    CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                      
                    !extern verify variables 
                      
                    !local verify variables 
                    CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
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
                LocalVar%WE%P = MATMUL(identity(3) - MATMUL(LocalVar%WE%K,H),LocalVar%WE%P)
                    END DO   
                    CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                    kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                    IF (check_status%rank==0) THEN 
                        WRITE (*, *) "identity : Time per call (usec): ", kgen_measure 
                    END IF   
                END IF   
                IF (kgen_warmupstage) THEN 
                END IF   
                IF (kgen_evalstage) THEN 
                END IF   

        ! Add RoutineName to error message


END SUBROUTINE windspeedestimator 
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks