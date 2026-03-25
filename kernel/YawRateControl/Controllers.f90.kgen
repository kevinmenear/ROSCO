!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-24 00:48:37 
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

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------  


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        ! Yaw rate controller
        !       Y_ControlMode = 0, No yaw control
        !       Y_ControlMode = 1, Yaw rate control using yaw drive
        ! TODO: Lots of R2D->D2R, this should be cleaned up.
        ! TODO: The constant offset implementation is sort of circular here as a setpoint is already being defined in SetVariablesSetpoints. This could also use cleanup

        USE rosco_types, ONLY: controlparameters, localvariables, objectinstances, debugvariables, errorvariables 
        USE rosco_types, ONLY: kr_rosco_types_controlparameters 
        USE rosco_types, ONLY: kr_rosco_types_localvariables 
        USE rosco_types, ONLY: kr_rosco_types_objectinstances 
        USE rosco_types, ONLY: kr_rosco_types_debugvariables 
        USE rosco_types, ONLY: kr_rosco_types_errorvariables 
        USE rosco_types, ONLY: kv_rosco_types_controlparameters 
        USE rosco_types, ONLY: kv_rosco_types_localvariables 
        USE rosco_types, ONLY: kv_rosco_types_objectinstances 
        USE rosco_types, ONLY: kv_rosco_types_debugvariables 
        USE rosco_types, ONLY: kv_rosco_types_errorvariables 
    
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*) ! The swap array, used to pass data to, and receive data from, the DLL controller.
    
        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
        TYPE(DebugVariables), INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT)       :: ErrVar
        ! Allocate Variables

        REAL(DbKi), SAVE :: NacVaneOffset                          ! For offset control
        INTEGER, SAVE :: YawState                               ! Yawing left(-1), right(1), or stopped(0)
        REAL(DbKi)       :: WindDirPlusOffset                     ! Instantaneous wind direction minus the assigned vane offset (deg)
        REAL(DbKi)       :: WindDirPlusOffsetCosF                 ! Time-filtered x-component of WindDirPlusOffset (deg)
        REAL(DbKi)       :: WindDirPlusOffsetSinF                 ! Time-filtered y-component of WindDirPlusOffset (deg)
        REAL(DbKi)       :: NacHeadingTarget                       ! Time-filtered wind direction minus the assigned vane offset (deg)
        REAL(DbKi), SAVE :: NacHeadingError                        ! Yaw error (deg)
        REAL(DbKi)       :: YawRateCom                             ! Commanded yaw rate (deg/s)
        REAL(DbKi)       :: deadband                               ! Allowable yaw error deadband (deg)
        REAL(DbKi)       :: Time                                   ! Current time
        INTEGER, SAVE :: Tidx                                   ! Index i: commanded yaw error is interpolated between i and i+1
        
        IF (CntrPar%Y_ControlMode == 1) THEN
            ! Compass wind directions in degrees

            LocalVar%WindDir = wrap_180(LocalVar%NacHeading + LocalVar%NacVane)
            ! Initialize

            IF (LocalVar%iStatus == 0) THEN
                YawState = 0
                Tidx = 1
            ENDIF
            ! Compute/apply offset
            
            IF (CntrPar%ZMQ_Mode == 1) THEN
                NacVaneOffset = LocalVar%ZMQ_YawOffset
            ELSE
                NacVaneOffset = CntrPar%Y_MErrSet ! (deg) # Offset from setpoint
            ENDIF
            ! Update filtered wind direction

            WindDirPlusOffset = wrap_180(LocalVar%WindDir + NacVaneOffset) ! (deg)
            WindDirPlusOffsetCosF = LPFilter(cos(WindDirPlusOffset*D2R), LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
            WindDirPlusOffsetSinF = LPFilter(sin(WindDirPlusOffset*D2R), LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
            NacHeadingTarget = wrap_180(atan2(WindDirPlusOffsetSinF, WindDirPlusOffsetCosF) * R2D) ! (deg)
            ! ---- Now get into the guts of the control ----
            ! Yaw error

            NacHeadingError = wrap_180(NacHeadingTarget - LocalVar%NacHeading)
            ! Check for deadband
			
            IF (LocalVar%WE_Vw_F .le. CntrPar%Y_uSwitch) THEN
                deadband = CntrPar%Y_ErrThresh(1)
            ELSE
                deadband = CntrPar%Y_ErrThresh(2)
            ENDIF
            ! yawing right

            IF (YawState == 1) THEN 
                IF (NacHeadingError .le. 0) THEN
                    ! stop yawing
                    YawRateCom = 0.0
                    YawState = 0 
                ELSE
                    ! persist
                    YawRateCom = CntrPar%Y_Rate
                    YawState = 1 
                ENDIF
            ! yawing left
            ELSEIF (YawState == -1) THEN 
                IF (NacHeadingError .ge. 0) THEN
                    ! stop yawing
                    YawRateCom = 0.0
                    YawState = 0 
                ELSE
                    ! persist
                    YawRateCom = -CntrPar%Y_Rate
                    YawState = -1 
                ENDIF
            ! Initiate yaw if outside yaw error threshold
            ELSE
                IF (NacHeadingError .gt. deadband) THEN
                    YawState = 1 ! yaw right
                ENDIF

                IF (NacHeadingError .lt. -deadband) THEN
                    YawState = -1 ! yaw left
                ENDIF

                YawRateCom = 0.0 ! if YawState is not 0, start yawing on the next time step
            ENDIF
            ! Output yaw rate command in rad/s

            avrSWAP(48) = YawRateCom * D2R
            ! If using open loop yaw rate control, overwrite controlled output
            ! Open loop yaw rate control - control input in rad/s

            IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_YawRate > 0)) THEN
                IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
                    avrSWAP(48) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_YawRate,LocalVar%OL_Index, ErrVar)
                ENDIF
            ENDIF
            ! Save for debug

            DebugVar%YawRateCom       = YawRateCom
            DebugVar%NacHeadingTarget = NacHeadingTarget
            DebugVar%NacVaneOffset    = NacVaneOffset
            DebugVar%YawState         = YawState
            DebugVar%Yaw_Err          = NacHeadingError
        END IF
    END SUBROUTINE YawRateControl
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


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers