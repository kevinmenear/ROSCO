!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-27 18:26:38 
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
    PUBLIC pitchcontrol 

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
SUBROUTINE pitchcontrol(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst, debugvar) 
    ! Blade pitch controller, generally maximizes rotor speed below rated (region 2) and regulates rotor speed above rated (region 3)
    !       PC_State = PC_State_Disabled (0), fix blade pitch to fine pitch angle (PC_FinePit)
    !       PC_State = PC_State_Disabled (1), is gain scheduled PI controller 
    USE rosco_types, ONLY: controlparameters, localvariables, objectinstances, debugvariables 
        ! Inputs
    USE kgen_utils_mod, ONLY: kgen_dp, kgen_array_sumcheck 
    USE kgen_utils_mod, ONLY: kgen_perturb_real 
    USE rosco_types, ONLY: kr_rosco_types_controlparameters 
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kr_rosco_types_debugvariables 
    USE rosco_types, ONLY: kr_rosco_types_objectinstances 
    USE rosco_types, ONLY: kv_rosco_types_controlparameters 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_debugvariables 
    USE rosco_types, ONLY: kv_rosco_types_objectinstances 
        
    TYPE(controlparameters), INTENT(INOUT) :: cntrpar 
    TYPE(localvariables), INTENT(INOUT) :: localvar 
    TYPE(objectinstances), INTENT(INOUT) :: objinst 
    TYPE(debugvariables), INTENT(INOUT) :: debugvar 
        ! Allocate Variables:


        ! ------- Blade Pitch Controller --------
        ! Load PC State
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
    TYPE(controlparameters) :: kgenref_cntrpar 
    TYPE(localvariables) :: kgenref_localvar 
    TYPE(objectinstances) :: kgenref_objinst 
    TYPE(debugvariables) :: kgenref_debugvar 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_controlparameters(kgenref_cntrpar, kgen_unit, "kgenref_cntrpar", .FALSE.) 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(kgenref_objinst, kgen_unit, "kgenref_objinst", .FALSE.) 
    CALL kr_rosco_types_debugvariables(kgenref_debugvar, kgen_unit, "kgenref_debugvar", .FALSE.) 


        ! Hold blade pitch at last value
        ! If:
        !   In pre-startup mode (before freewheeling)


        ! Compute (interpolate) the gains based on previously commanded blade pitch angles and lookup table:
        
        ! Compute the collective pitch command associated with the proportional and integral gains:
        
        ! Find individual pitch control contribution


        ! Include tower fore-aft tower vibration damping control
        

        ! Pitch Saturation
        

        ! FloatingFeedback
        

        ! Saturate collective pitch commands:
        
        ! Combine and saturate all individual pitch commands in software


        ! Open Loop control, use if
        !   Open loop mode active         Using OL blade pitch control      


        ! Active wake control

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
            CALL ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
            IF (kgen_mainstage) THEN 
                  
                !verify init 
                CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=1) 
                CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                  
                !extern verify variables 
                  
                !local verify variables 
                CALL kv_rosco_types_controlparameters("cntrpar", check_status, cntrpar, kgenref_cntrpar) 
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
            CALL ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
                END DO   
                CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                IF (check_status%rank==0) THEN 
                    WRITE (*, *) "ActiveWakeControl : Time per call (usec): ", kgen_measure 
                END IF   
            END IF   
            IF (kgen_warmupstage) THEN 
            END IF   
            IF (kgen_evalstage) THEN 
            END IF   
        ! Shutdown


        ! Place pitch actuator here, so it can be used with or without open-loop
        
       

        ! Hardware saturation: using CntrPar%PC_MinPit


        ! Add pitch actuator fault for blade K


        ! Command the pitch demanded from the last
        ! call to the controller (See Appendix A of Bladed User's Guide):

        ! Add RoutineName to error message


END SUBROUTINE pitchcontrol 
!-------------------------------------------------------------------------------------------------------------------------------  


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


    SUBROUTINE ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        ! Active wake controller
        !       AWC_Mode = 0, No active wake control
        !       AWC_Mode = 1, SNL active wake control
        !       AWC_Mode = 2, Coleman Transform-based active wake control
        !       AWC_Mode = 3, Closed-loop Proportional-integral (PI) active wake control
        !       AWC_Mode = 4, Closed-loop Proportional-resonant (PR) active wake control
        !       AWC_Mode = 5, Strouhal transformation based closed-loop active wake control
        USE rosco_types, ONLY: controlparameters, localvariables, debugvariables, objectinstances 
        USE rosco_types, ONLY: kr_rosco_types_controlparameters 
        USE rosco_types, ONLY: kr_rosco_types_localvariables 
        USE rosco_types, ONLY: kr_rosco_types_debugvariables 
        USE rosco_types, ONLY: kr_rosco_types_objectinstances 
        USE rosco_types, ONLY: kv_rosco_types_controlparameters 
        USE rosco_types, ONLY: kv_rosco_types_localvariables 
        USE rosco_types, ONLY: kv_rosco_types_debugvariables 
        USE rosco_types, ONLY: kv_rosco_types_objectinstances 

        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(DebugVariables), INTENT(INOUT)       :: DebugVar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
        ! Local vars

        REAL(DbKi), PARAMETER      :: phi1 = 0.0                       ! Phase difference from first to first blade
        REAL(DbKi), PARAMETER      :: phi2 = 2.0/3.0*PI                ! Phase difference from first to second blade
        REAL(DbKi), PARAMETER      :: phi3 = 4.0/3.0*PI                ! Phase difference from first to third blade
        REAL(DbKi), DIMENSION(3)      :: AWC_angle
        COMPLEX(DbKi), DIMENSION(3)   :: AWC_complexangle
        ! COMPLEX(DbKi)              :: complexI = (0.0, 1.0)  ! VIT: removed (complex split)
        INTEGER(IntKi)             :: Imode, K                         ! Index used for looping through AWC modes, blades
        REAL(DbKi)                 :: clockang                         ! Clock angle for AWC pitching
        REAL(DbKi)                 :: omega                            ! angular frequency for AWC pitching in Hz
        REAL(DbKi)                 :: amp                              ! amplitude for AWC pitching in degrees
        REAL(DbKi), DIMENSION(2)   :: AWC_TiltYaw = [0.0, 0.0]         ! AWC tilt and yaw pitch signals
        REAL(DbKi), DIMENSION(2)   :: Error = [0.0, 0.0]               ! Error in transformed tilt and yaw signals
        REAL(DbKi), DIMENSION(2)   :: FixedFrameM                      ! Measured tilt moment
        REAL(DbKi)                 :: StrAzimuth                       ! Strouhal transformed "azimuth" angle
        REAL(DbKi)                 :: StartTime = 0.0                  ! Start time of closed-loop AWC
        ! Compute the AWC pitch settings, complex number approach


        IF (CntrPar%AWC_Mode == 1) THEN

            LocalVar%AWC_complexangle_re = 0.0D0
            LocalVar%AWC_complexangle_im = 0.0D0

            DO Imode = 1,CntrPar%AWC_NumModes
                clockang = CntrPar%AWC_clockangle(Imode)*PI/180.0_DbKi
                omega = CntrPar%AWC_freq(Imode)*PI*2.0_DbKi
                AWC_angle(1) = omega * LocalVar%Time - CntrPar%AWC_n(Imode) * (LocalVar%Azimuth + phi1 + clockang)
                AWC_angle(2) = omega * LocalVar%Time - CntrPar%AWC_n(Imode) * (LocalVar%Azimuth + phi2 + clockang)
                AWC_angle(3) = omega * LocalVar%Time - CntrPar%AWC_n(Imode) * (LocalVar%Azimuth + phi3 + clockang)
                ! Add the forcing contribution to LocalVar%AWC_complexangle
                amp = CntrPar%AWC_amp(Imode)*PI/180.0_DbKi
                DO K = 1,LocalVar%NumBl ! Loop through all blades
                    LocalVar%AWC_complexangle_re(K) = LocalVar%AWC_complexangle_re(K) + amp * COS(AWC_angle(K))
                    LocalVar%AWC_complexangle_im(K) = LocalVar%AWC_complexangle_im(K) + amp * SIN(AWC_angle(K))
                END DO
            END DO

            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%AWC_complexangle_re(K)
            END DO
        ! Open-loop Coleman transform method

        ELSEIF (CntrPar%AWC_Mode == 2) THEN
            ! Calculate reference OL blade signals

            AWC_TiltYaw = [0.0, 0.0]
            DO Imode = 1,CntrPar%AWC_NumModes
                AWC_TiltYaw(Imode) = D2R*CntrPar%AWC_amp(Imode)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(Imode) + CntrPar%AWC_clockangle(Imode)*D2R)
                IF (CntrPar%AWC_NumModes == 1) THEN
                    AWC_TiltYaw(2) = D2R*CntrPar%AWC_amp(1)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(1) + 2*CntrPar%AWC_clockangle(1)*D2R)
                ENDIF
                ! Inverse Coleman Transformation with phase offset
                
                CALL ColemanTransformInverse(AWC_TiltYaw(1), AWC_TiltYaw(2), LocalVar%Azimuth, CntrPar%AWC_harmonic(Imode), CntrPar%AWC_phaseoffset*D2R, AWC_angle)
            END DO

            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_angle(K)
            END DO
            ! DEBUG VARIABLES

            DebugVar%axisTilt_2P = AWC_TiltYaw(1)
            DebugVar%axisYaw_2P = AWC_TiltYaw(2)
            CALL ColemanTransform(LocalVar%BlPitch, LocalVar%Azimuth, CntrPar%AWC_harmonic(1), AWC_TiltYaw(1), AWC_TiltYaw(2))

            DebugVar%axisTilt_1P = AWC_TiltYaw(1)
            DebugVar%axisYaw_1P = AWC_TiltYaw(2)
        ! Closed-loop PI / PR controller
        
        ELSEIF ((CntrPar%AWC_Mode == 3) .OR. (CntrPar%AWC_Mode == 4)) THEN
            !! If AWC_harmonic > 0, use AWC_NumModes=2

        
            CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, CntrPar%AWC_harmonic(1), FixedFrameM(1), FixedFrameM(2))

            IF (CntrPar%AWC_harmonic(1) == 0) THEN
                ! Calculate mean moments, subtract later to get zero-mean tilt and yaw
                LocalVar%TiltMean = LocalVar%TiltMean + FixedFrameM(1)
                StartTime = 1/CntrPar%AWC_freq(1)
            ENDIF

            IF (LocalVar%Time .GT. StartTime) THEN
                DO Imode = 1,CntrPar%AWC_NumModes
                    Error(Imode) = CntrPar%AWC_amp(Imode)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(Imode) + CntrPar%AWC_clockangle(Imode)*D2R) &
                                        + (FixedFrameM(Imode) - LocalVar%TiltMean/(LocalVar%n_DT+1))

                    IF (CntrPar%AWC_Mode == 4) THEN
                        AWC_TiltYaw(Imode) = ResController(Error(Imode), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), CntrPar%AWC_freq(Imode), & 
                                                            CntrPar%PC_MinPit, CntrPar%PC_MaxPit, LocalVar%DT, LocalVar%resP, (LocalVar%restart /= 0), objInst%instRes)
                    ELSE
                        AWC_TiltYaw(Imode) = PIController(Error(Imode), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), CntrPar%PC_MinPit, CntrPar%PC_MaxPit, &
                                                            LocalVar%DT, 0.0_DbKi, LocalVar%piP, (LocalVar%restart /= 0), objInst%instPI)
                    ENDIF
                ENDDO
            ENDIF
            ! Pass tilt and yaw axis through the inverse Coleman transform to get the commanded pitch angles

            CALL ColemanTransformInverse(AWC_TiltYaw(1), AWC_TiltYaw(2), &
                                         LocalVar%Azimuth, CntrPar%AWC_harmonic(1), CntrPar%AWC_phaseoffset*D2R, AWC_angle)
            
            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                IF (CntrPar%AWC_harmonic(1) == 0) THEN
                    LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_TiltYaw(1)
                ELSE
                    LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_angle(K)
                ENDIF
            END DO
            ! DEBUG VARIABLES

            DebugVar%axisTilt_1P = AWC_TiltYaw(1)
            DebugVar%axisYaw_1P = -FixedFrameM(1) + LocalVar%TiltMean/(LocalVar%n_DT+1)
            DebugVar%axisTilt_2P = CntrPar%AWC_amp(1)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(1) + CntrPar%AWC_clockangle(1)*D2R)
            DebugVar%axisYaw_2P = Error(1)
        ! Closed-loop Strouhal transform method

        ELSEIF (CntrPar%AWC_Mode == 5) THEN

            StrAzimuth = wrap_360(360*LocalVar%Time*CntrPar%AWC_freq(1))*D2R

            CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, CntrPar%AWC_harmonic(1), FixedFrameM(1), FixedFrameM(2))
            ! Calculate mean tilt and yaw moments to subtract
            
            LocalVar%TiltMean = LocalVar%TiltMean + FixedFrameM(1)
            LocalVar%YawMean = LocalVar%YawMean + FixedFrameM(2)
            ! Calculate error with zero-mean moments

            Error(1) = CntrPar%AWC_amp(1) + sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)*(FixedFrameM(1) - LocalVar%TiltMean/(LocalVar%n_DT+1)) &
                        + sin(StrAzimuth + CntrPar%AWC_clockangle(2)*D2R)*(FixedFrameM(2) - LocalVar%YawMean/(LocalVar%n_DT+1)) 
            ! PI Control

            IF (LocalVar%Time .GT. 1/CntrPar%AWC_freq(1)) THEN
                AWC_TiltYaw(1) = PIController(Error(1), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), &
                                    CntrPar%PC_MinPit, CntrPar%PC_MaxPit, LocalVar%DT, 0.0_DbKi, LocalVar%piP, (LocalVar%restart /= 0), objInst%instPI)
            ENDIF
            ! Pass tilt and yaw axis through the inverse Strouhal + Coleman transform to get the commanded pitch angles

            CALL ColemanTransformInverse(sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)*AWC_TiltYaw(1), & ! Tilt signal (inverse Str transform)
                                            sin(StrAzimuth + CntrPar%AWC_clockangle(2)*D2R)*AWC_TiltYaw(1), & ! Yaw signal (inverse Str transform)
                                            LocalVar%Azimuth, CntrPar%AWC_harmonic(1), CntrPar%AWC_phaseoffset*D2R, AWC_angle)

            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_angle(K)
            END DO
            ! DEBUG VARIABLES

            DebugVar%axisTilt_1P = sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)*AWC_TiltYaw(1)
            DebugVar%axisYaw_1P = -FixedFrameM(1) + LocalVar%TiltMean/(LocalVar%n_DT+1)
            DebugVar%axisTilt_2P = CntrPar%AWC_amp(1)*sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)
            DebugVar%axisYaw_2P = Error(1)


        ENDIF

    END SUBROUTINE ActiveWakeControl
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


!-------------------------------------------------------------------------------------------------------------------------------


        !-------------------------------------------------------------------------------------------------------------------------------

    REAL(DbKi) FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst)
        USE rosco_types, ONLY: resparams 
    ! PI controller, with output saturation
        USE rosco_types, ONLY: kr_rosco_types_resparams 
        USE rosco_types, ONLY: kv_rosco_types_resparams 


        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: freq
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)     :: inst
        TYPE(resParams), INTENT(INOUT)    :: resP
        LOGICAL,    INTENT(IN)            :: reset
        ! Allocate local variables
        REAL(DbKi)                        :: omega                                        ! Frequency
        REAL(DbKi)                        :: a0, a1, a2, b0, b1, b2

        omega = 2*PI*freq
        !! Tustin RC

        b0 = 4+omega**2*DT**2
        b1 = -8+2*omega**2*DT**2
        b2 = 4+omega**2*DT**2
        a0 = b0*kp + 2*DT*ki
        a1 = b1*kp 
        a2 = b2*kp - 2*DT*ki
        ! Initialize persistent variables/arrays, and set initial condition for integrator term
        IF (reset) THEN
            resP%res_OutputSignalLast1(inst)  = 0
            resP%res_OutputSignalLast2(inst)  = 0
            resP%res_InputSignalLast1(inst)   = 0
            resP%res_InputSignalLast2(inst)   = 0
        ELSE
            ResController = 1/b0*( -b1*resP%res_OutputSignalLast1(inst) - b2*resP%res_OutputSignalLast2(inst) &
                                    + a0*error + a1*resP%res_InputSignalLast1(inst) + a2*resP%res_InputSignalLast2(inst))
            ResController = saturate(ResController, minValue, maxValue)
            ! Save signals for next time step
        
            resP%res_InputSignalLast2(inst)   = resP%res_InputSignalLast1(inst)
            resP%res_InputSignalLast1(inst)   = error
            resP%res_OutputSignalLast2(inst)  = resP%res_OutputSignalLast1(inst)
            resP%res_OutputSignalLast1(inst)  = ResController
        END IF
        inst = inst + 1
        
    END FUNCTION ResController
!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers