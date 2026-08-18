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

    USE, INTRINSIC :: ISO_C_Binding
    USE Functions
    USE Filters
    USE ControllerBlocks

    IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of PIController
    INTERFACE
        FUNCTION picontroller_c(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst) BIND(C, NAME='picontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: I0
            TYPE(C_PTR), VALUE :: piP
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: picontroller_c
        END FUNCTION picontroller_c
    END INTERFACE


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
            INTEGER(C_INT), VALUE :: reset
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: LocalVar
            REAL(C_DOUBLE) :: pidcontroller_c
        END FUNCTION pidcontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PIIController
    INTERFACE
        FUNCTION piicontroller_c(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) BIND(C, NAME='piicontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: error2
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: ki2
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: I0
            TYPE(C_PTR), VALUE :: piP
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: piicontroller_c
        END FUNCTION piicontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ResController
    INTERFACE
        FUNCTION rescontroller_c(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) BIND(C, NAME='rescontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: freq
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            TYPE(C_PTR), VALUE :: resP
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: rescontroller_c
        END FUNCTION rescontroller_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of StructuralControl
    INTERFACE
        SUBROUTINE structuralcontrol_c(avrSWAP, CntrPar, LocalVar, objInst, ErrVar) BIND(C, NAME='structuralcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE structuralcontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of YawRateControl
    INTERFACE
        SUBROUTINE yawratecontrol_c(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='yawratecontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE yawratecontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ActiveWakeControl
    INTERFACE
        SUBROUTINE activewakecontrol_c(CntrPar, LocalVar, DebugVar, objInst) BIND(C, NAME='activewakecontrol_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE activewakecontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of CableControl
    INTERFACE
        SUBROUTINE cablecontrol_c(avrSWAP, CntrPar, LocalVar, objInst, ErrVar) BIND(C, NAME='cablecontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE cablecontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of FlapControl
    INTERFACE
        SUBROUTINE flapcontrol_c(avrSWAP, CntrPar, LocalVar, objInst) BIND(C, NAME='flapcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE flapcontrol_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of FloatingFeedback
    INTERFACE
        FUNCTION floatingfeedback_c(LocalVar, CntrPar, objInst, ErrVar) BIND(C, NAME='floatingfeedback_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: floatingfeedback_c
        END FUNCTION floatingfeedback_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ForeAftDamping
    INTERFACE
        SUBROUTINE foreaftdamping_c(CntrPar, LocalVar, objInst) BIND(C, NAME='foreaftdamping_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE foreaftdamping_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of IPC
    INTERFACE
        SUBROUTINE ipc_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='ipc_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE ipc_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    ! Blade pitch controller, generally maximizes rotor speed below rated (region 2) and regulates rotor speed above rated (region 3)
    !       PC_State = PC_State_Disabled (0), fix blade pitch to fine pitch angle (PC_FinePit)
    !       PC_State = PC_State_Disabled (1), is gain scheduled PI controller 
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        
        ! Inputs
        REAL(ReKi),              INTENT(INOUT)       :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from the DLL controller.
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        ! Allocate Variables:
        INTEGER(IntKi)                                  :: K            ! Index used for looping through blades.

        CHARACTER(*),               PARAMETER           :: RoutineName = 'PitchControl'

        ! ------- Blade Pitch Controller --------
        ! Load PC State
        IF (LocalVar%PC_State == PC_State_Enabled) THEN ! PI BldPitch control
            LocalVar%PC_MaxPit = CntrPar%PC_MaxPit
        ELSE ! debug mode, fix at fine pitch
            LocalVar%PC_MaxPit = CntrPar%PC_FinePit
        END IF

        ! Hold blade pitch at last value
        ! If:
        !   In pre-startup mode (before freewheeling)
        IF ((CntrPar%SU_Mode > 0) .AND. (LocalVar%SU_Stage == -1)) THEN
            LocalVar%PC_MaxPit = LocalVar%BlPitchCMeas
            LocalVar%PC_MinPit = LocalVar%BlPitchCMeas
        END IF
        
        ! Compute (interpolate) the gains based on previously commanded blade pitch angles and lookup table:
        LocalVar%PC_KP = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KP, LocalVar%BlPitchCMeasF, ErrVar) ! Proportional gain
        LocalVar%PC_KI = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KI, LocalVar%BlPitchCMeasF, ErrVar) ! Integral gain
        LocalVar%PC_KD = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KD, LocalVar%BlPitchCMeasF, ErrVar) ! Derivative gain
        LocalVar%PC_TF = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_TF, LocalVar%BlPitchCMeasF, ErrVar) ! TF gains (derivative filter) !NJA - need to clarify
        
        ! Compute the collective pitch command associated with the proportional and integral gains:
        LocalVar%PC_PitComT = PIController(LocalVar%PC_SpdErr, LocalVar%PC_KP, LocalVar%PC_KI, LocalVar%PC_MinPit, LocalVar%PC_MaxPit, LocalVar%DT, LocalVar%BlPitch(1), LocalVar%piP, LocalVar%restart, objInst%instPI)
        DebugVar%PC_PICommand = LocalVar%PC_PitComT

        ! Find individual pitch control contribution
        IF ((CntrPar%IPC_ControlMode >= 1) .OR. (CntrPar%Y_ControlMode == 2)) THEN
            CALL IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        ELSE
            LocalVar%IPC_PitComF = 0.0 ! THIS IS AN ARRAY!!
        END IF
        
        ! Include tower fore-aft tower vibration damping control
        IF (CntrPar%TD_Mode > 0) THEN
            CALL ForeAftDamping(CntrPar, LocalVar, objInst)
        ELSE
            LocalVar%FA_PitCom = 0.0 ! THIS IS AN ARRAY!!
        ENDIF
        
        ! Pitch Saturation
        IF (CntrPar%PS_Mode > 0) THEN
            LocalVar%PC_MinPit = PitchSaturation(LocalVar,CntrPar,objInst,DebugVar, ErrVar)
            LocalVar%PC_MinPit = max(LocalVar%PC_MinPit, CntrPar%PC_FinePit)
        ELSE
            LocalVar%PC_MinPit = CntrPar%PC_FinePit
        ENDIF
        DebugVar%PC_MinPit = LocalVar%PC_MinPit
        
        ! FloatingFeedback
        IF (CntrPar%Fl_Mode > 0) THEN
            LocalVar%Fl_PitCom = FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar)
            DebugVar%FL_PitCom = LocalVar%Fl_PitCom
            LocalVar%PC_PitComT = LocalVar%PC_PitComT + LocalVar%Fl_PitCom
        ENDIF
        
        ! Saturate collective pitch commands:
        LocalVar%PC_PitComT = saturate(LocalVar%PC_PitComT, LocalVar%PC_MinPit, CntrPar%PC_MaxPit)                    ! Saturate the overall command using the pitch angle limits
        LocalVar%PC_PitComT = ratelimit(LocalVar%PC_PitComT, CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL,LocalVar%BlPitchCMeas) ! Saturate the overall command of blade K using the pitch rate limit
        LocalVar%PC_PitComT_Last = LocalVar%PC_PitComT

        ! Combine and saturate all individual pitch commands in software
        DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
            LocalVar%PitCom(K) = LocalVar%PC_PitComT + LocalVar%FA_PitCom(K) 
            LocalVar%PitCom(K) = saturate(LocalVar%PitCom(K), LocalVar%PC_MinPit, CntrPar%PC_MaxPit)                    ! Saturate the command using the pitch saturation limits
            LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%IPC_PitComF(K)                                          ! Add IPC
            
            ! Hard IPC saturation by peak shaving limit
            IF (CntrPar%IPC_SatMode == 1) THEN
                LocalVar%PitCom(K) = saturate(LocalVar%PitCom(K), LocalVar%PC_MinPit, CntrPar%PC_MaxPit)  
            END IF
            
            ! Add ZeroMQ pitch commands
            LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%ZMQ_PitOffset(K)

            ! Rate limit                  
            LocalVar%PitCom(K) = ratelimit(LocalVar%PitCom(K), CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL,LocalVar%BlPitch(K)) ! Saturate the overall command of blade K using the pitch rate limit
        END DO 

        ! Open Loop control, use if
        !   Open loop mode active         Using OL blade pitch control      
        IF (CntrPar%OL_Mode > 0) THEN
            IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN    ! Time > first open loop breakpoint
                IF (CntrPar%Ind_BldPitch(1) > 0) THEN
                    LocalVar%PitCom(1) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_BldPitch1,LocalVar%OL_Index, ErrVar)
                ENDIF

                IF (CntrPar%Ind_BldPitch(2) > 0) THEN
                    LocalVar%PitCom(2) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_BldPitch2,LocalVar%OL_Index, ErrVar)
                ENDIF

                IF (CntrPar%Ind_BldPitch(3) > 0) THEN
                    LocalVar%PitCom(3) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_BldPitch3,LocalVar%OL_Index, ErrVar)
                ENDIF
            ENDIF
        ENDIF

        ! Active wake control
        IF (CntrPar%AWC_Mode > 0) THEN
            CALL ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        ENDIF

        ! Shutdown
        IF (LocalVar%SD_Trigger == 0) THEN
            LocalVar%PitCom_SD = LocalVar%PitCom
        ! If shutdown is not triggered, PitCom_SD tracks PitCom.
        ELSE
            IF (CntrPar%SD_Method == 1 .OR. CntrPar%SD_Method == 2) THEN
                DO K = 1,LocalVar%NumBl
                    LocalVar%PitCom_SD(K) = LocalVar%PitCom_SD(K) + LocalVar%SD_MaxPitchRate*LocalVar%DT
                END DO
            ENDIF
            LocalVar%PitCom = LocalVar%PitCom_SD
            ! When shutdown is triggered (SD_Trigger \=0), pitch to feather.
            ! Note that in some instances (like a downwind rotor), we may want to pitch to a stall angle.
        ENDIF
        
       
        ! Place pitch actuator here, so it can be used with or without open-loop
        DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
            IF (CntrPar%PA_Mode > 0) THEN
                IF (CntrPar%PA_Mode == 1) THEN
                    LocalVar%PitComAct(K) = LPFilter(LocalVar%PitCom(K), LocalVar%DT, CntrPar%PA_CornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
                ELSE IF (CntrPar%PA_Mode == 2) THEN
                    LocalVar%PitComAct(K) = SecLPFilter(LocalVar%PitCom(K),LocalVar%DT,CntrPar%PA_CornerFreq,CntrPar%PA_Damping,LocalVar%FP,LocalVar%iStatus,LocalVar%restart,objInst%instSecLPF)
                END IF  
            ELSE
                LocalVar%PitComAct(K) = LocalVar%PitCom(K)
            ENDIF
        END DO

        ! Hardware saturation: using CntrPar%PC_MinPit
        DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
            ! Saturate the pitch command using the overall (hardware) limit
            LocalVar%PitComAct(K) = saturate(LocalVar%PitComAct(K), CntrPar%PC_MinPit, CntrPar%PC_MaxPit)
            ! Saturate the overall command of blade K using the pitch rate limit
            LocalVar%PitComAct(K) = ratelimit(LocalVar%PitComAct(K), CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL,LocalVar%BlPitch(K)) ! Saturate the overall command of blade K using the pitch rate limit
        END DO

        ! Add pitch actuator fault for blade K
        IF (CntrPar%PF_Mode == 1) THEN
            DO K = 1, LocalVar%NumBl
                ! This assumes that the pitch actuator fault overides the Hardware saturation
                LocalVar%PitComAct(K) = LocalVar%PitComAct(K) + CntrPar%PF_Offsets(K)
            END DO
        ! Blade pitch stuck at last value
        ELSEIF (CntrPar%PF_Mode == 2) THEN
            DO K = 1, LocalVar%NumBl
                IF (LocalVar%Time > CntrPar%PF_TimeStuck(K)) THEN
                    LocalVar%PitComAct(K) = LocalVar%BlPitch(K)
                END IF
            END DO
        END IF

        ! Command the pitch demanded from the last
        ! call to the controller (See Appendix A of Bladed User's Guide):
        avrSWAP(42) = LocalVar%PitComAct(1)   ! Use the command angles of all blades if using individual pitch
        avrSWAP(43) = LocalVar%PitComAct(2)   ! "
        avrSWAP(44) = LocalVar%PitComAct(3)   ! "
        avrSWAP(45) = LocalVar%PitComAct(1)   ! Use the command angle of blade 1 if using collective pitch

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
    END SUBROUTINE PitchControl
!-------------------------------------------------------------------------------------------------------------------------------  
    SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
    ! Generator torque controller
    !       VS_State = VS_State_Error             (0), Error state, for debugging purposes, GenTq = VS_RtTq
    !       VS_State = VS_State_Region_1_5        (1), Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
    !       VS_State = VS_State_Region_2          (2), Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
    !       VS_State = VS_State_Region_2_5        (3), Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
    !       VS_State = VS_State_Region_3_ConstTrq (4), above-rated operation using pitch control (constant torque mode)
    !       VS_State = VS_State_Region_3_ConstPwr (5), above-rated operation using pitch and torque control (constant power mode)
    !       VS_State = VS_State_PI                (6), Tip-Speed-Ratio tracking PI controller (ignore state machine)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        ! Inputs
        REAL(ReKi),                 INTENT(INOUT)       :: avrSWAP(*)    ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        CHARACTER(*),               PARAMETER           :: RoutineName = 'VariableSpeedControl'

        ! Allocate Variables

        ! -------- Variable-Speed Torque Controller --------

        ! Pre-compute generator torque values for K*Omega^2 and constant power
        LocalVar%VS_KOmega2_GenTq = CntrPar%VS_Rgn2K*LocalVar%GenSpeedF*LocalVar%GenSpeedF
        LocalVar%VS_ConstPwr_GenTq = (CntrPar%VS_RtPwr/(CntrPar%VS_GenEff/100.0))/LocalVar%GenSpeedF * LocalVar%PRC_R_Torque

        ! Determine maximum torque saturation limit, VS_MaxTq
        IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN 
            ! Variable pitch mode        
            IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN
                LocalVar%VS_MaxTq = min(LocalVar%VS_ConstPwr_GenTq, CntrPar%VS_MaxTq)
            ELSE
                LocalVar%VS_MaxTq = CntrPar%VS_RtTq * LocalVar%PRC_R_Torque
            END IF
        ELSE   
            ! Constant pitch, max torque is control parameter
            LocalVar%VS_MaxTq = CntrPar%VS_MaxTq  
        ENDIF 

        ! Optimal Tip-Speed-Ratio tracking controller (reference generated in subroutine ComputeVariablesSetpoints)
        IF ((CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) .OR. \
            (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) .OR. \
            (CntrPar%VS_ControlMode == VS_Mode_Torque_TSR)) THEN


            ! PI controller
            LocalVar%GenTq = PIController( &
                                        LocalVar%VS_SpdErr, &
                                        CntrPar%VS_KP(1), &
                                        CntrPar%VS_KI(1), &
                                        CntrPar%VS_MinTq, LocalVar%VS_MaxTq, &
                                        LocalVar%DT, LocalVar%VS_LastGenTrq, LocalVar%piP, LocalVar%restart, objInst%instPI)

            ! Saturate control input to Region 3 constant-power value if FBP mode is set to constant-power overspeed (no need for explicit transition region)
            IF (CntrPar%VS_FBP == VS_FBP_Power_Overspeed) THEN
                LocalVar%GenTq = MIN(LocalVar%VS_ConstPwr_GenTq, LocalVar%GenTq)
            ENDIF

        ! K*Omega^2 control law with PI torque control in transition regions
        ELSEIF (CntrPar%VS_ControlMode == VS_Mode_KOmega) THEN
            ! Update PI loops for region 1.5 and 2.5 PI control
            LocalVar%GenArTq = PIController(LocalVar%VS_SpdErrAr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MaxOMTq, CntrPar%VS_ArSatTq, LocalVar%DT, CntrPar%VS_MaxOMTq, LocalVar%piP, LocalVar%restart, objInst%instPI)
            LocalVar%GenBrTq = PIController(LocalVar%VS_SpdErrBr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MinTq, CntrPar%VS_MinOMTq, LocalVar%DT, CntrPar%VS_MinOMTq, LocalVar%piP, LocalVar%restart, objInst%instPI)

            ! State machine if switching to blade pitch control
            IF (LocalVar%VS_State == VS_State_Region_1_5) THEN ! Region 1.5
                LocalVar%GenTq = LocalVar%GenBrTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_2) THEN ! Region 2
                LocalVar%GenTq = LocalVar%VS_KOmega2_GenTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_2_5) THEN ! Region 2.5
                LocalVar%GenTq = LocalVar%GenArTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_3_ConstTrq) THEN ! Region 3, constant torque
                LocalVar%GenTq = CntrPar%VS_RtTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_3_ConstPwr) THEN ! Region 3, constant power
                LocalVar%GenTq = LocalVar%VS_ConstPwr_GenTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_3_FBP) THEN ! Region 3, fixed blade pitch
                ! Constant power overspeed
                IF (CntrPar%VS_FBP == VS_FBP_Power_Overspeed) THEN
                    ! K*Omega^2 in Region 2 or constant power overspeed in Region 3
                    LocalVar%GenTq = MIN(LocalVar%VS_ConstPwr_GenTq, LocalVar%VS_KOmega2_GenTq)
                ! Reference-tracking in Region 3
                ELSEIF ((CntrPar%VS_FBP == VS_FBP_WSE_Ref) .OR. (CntrPar%VS_FBP == VS_FBP_Torque_Ref)) THEN
                    LocalVar%GenTq = LocalVar%GenArTq
                ENDIF
            END IF

        ELSE        ! VS_ControlMode of 0
            LocalVar%GenTq = 0
        ENDIF
        
        
        ! Shutdown
        IF (LocalVar%SD_Trigger == 0) THEN
            LocalVar%GenTq_SD = LocalVar%GenTq
        ELSE 
            IF (CntrPar%SD_Method == 1 .OR. CntrPar%SD_Method == 2) THEN
                LocalVar%GenTq_SD = LocalVar%GenTq_SD - LocalVar%SD_MaxTorqueRate*LocalVar%DT
                LocalVar%GenTq_SD = saturate(LocalVar%GenTq_SD, CntrPar%VS_MinTq, CntrPar%VS_MaxTq)
            ENDIF
            LocalVar%GenTq = LocalVar%GenTq_SD
        ENDIF

        ! Saturate based on most stringent defined maximum
        LocalVar%GenTq = saturate(LocalVar%GenTq, CntrPar%VS_MinTq, MIN(CntrPar%VS_MaxTq, LocalVar%VS_MaxTq))

        ! Saturate the commanded torque using the torque rate limit
        LocalVar%GenTq = ratelimit(LocalVar%GenTq, -CntrPar%VS_MaxRat, CntrPar%VS_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL)    ! Saturate the command using the torque rate limit

        ! Open loop torque control
        IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_GenTq > 0)) THEN
            ! Get current OL GenTq, applies for OL_Mode 1 and 2
            IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
                LocalVar%GenTq = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_GenTq,LocalVar%OL_Index,ErrVar)
            ENDIF
            
            ! Azimuth tracking control
            IF (CntrPar%OL_Mode == 2) THEN
                
                ! Push, pop and unwrap azimuth buffer 
                ! Initialize
                IF (LocalVar%iStatus == 0) THEN
                    LocalVar%AzBuffer(1) = LocalVar%Azimuth
                    LocalVar%AzBuffer(2) = LocalVar%Azimuth
                ENDIF
                LocalVar%AzBuffer(1) = LocalVar%AzBuffer(2)
                LocalVar%AzBuffer(2) = LocalVar%Azimuth
                LocalVar%AzBuffer = UNWRAP(LocalVar%AzBuffer, ErrVar)
                LocalVar%AzUnwrapped = LocalVar%AzBuffer(2)

                ! Current desired Azimuth, error
                LocalVar%OL_Azimuth = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_Azimuth,LocalVar%Time,ErrVar)
                LocalVar%AzError = LocalVar%OL_Azimuth - LocalVar%AzUnwrapped 

                LocalVar%GenTqAz = PIDController(LocalVar%AzError, CntrPar%RP_Gains(1), CntrPar%RP_Gains(2), CntrPar%RP_Gains(3), CntrPar%RP_Gains(4), -LocalVar%VS_MaxTq * 2, LocalVar%VS_MaxTq * 2, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst, LocalVar)
                LocalVar%GenTq = LocalVar%GenTq + LocalVar%GenTqAz

            ENDIF

        ENDIF

        ! Reset the value of LocalVar%VS_LastGenTrq to the current values:
        LocalVar%VS_LastGenTrq = LocalVar%GenTq
        LocalVar%VS_LastGenPwr = LocalVar%VS_GenPwr
        
        ! Set the command generator torque (See Appendix A of Bladed User's Guide):
        avrSWAP(47) = MAX(0.0_DbKi, LocalVar%VS_LastGenTrq)  ! Demanded generator torque, prevent negatives.

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE VariableSpeedControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL yawratecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE YawRateControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL ipc_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE IPC
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ForeAftDamping(CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL foreaftdamping_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE ForeAftDamping
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar) RESULT(FloatingFeedback_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: FloatingFeedback_result
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        FloatingFeedback_result = REAL(floatingfeedback_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION FloatingFeedback
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL flapcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE FlapControl


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL activewakecontrol_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(DebugVar), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE ActiveWakeControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE CableControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL cablecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE CableControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StructuralControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL structuralcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE StructuralControl
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: I0
        TYPE(PIPARAMS), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8) :: PIController_result
        PIController_result = REAL(picontroller_c(error, kp, ki, minValue, maxValue, DT, I0, C_LOC(piP), MERGE(1_C_INT, 0_C_INT, reset), inst), 8)
    END FUNCTION PIController


!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar) RESULT(PIDController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams, LocalVariables, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: kd
        REAL(8), INTENT(IN) :: tf
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: I0
        TYPE(PIPARAMS), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        REAL(8) :: PIDController_result
        TYPE(C_PTR) :: piP_cptr
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        ! Resolve aliasing: an argument that IS a component of a view-type
        ! argument must be written through the VIEW, or the view copy-back
        ! afterwards would discard those writes (see _aliased_component_args).
        IF (C_ASSOCIATED(C_LOC(piP), C_LOC(LocalVar%piP))) THEN
            piP_cptr = C_LOC(LocalVar_view%piP)
        ELSE
            piP_cptr = C_LOC(piP)
        END IF
        PIDController_result = REAL(pidcontroller_c(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP_cptr, MERGE(1_C_INT, 0_C_INT, reset), C_LOC(objInst), C_LOC(LocalVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END FUNCTION PIDController

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: error2
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: ki2
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: I0
        TYPE(PIPARAMS), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8) :: PIIController_result
        PIIController_result = REAL(piicontroller_c(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, C_LOC(piP), MERGE(1_C_INT, 0_C_INT, reset), inst), 8)
    END FUNCTION PIIController

        !-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) RESULT(ResController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : resParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: error
        REAL(8), INTENT(IN) :: kp
        REAL(8), INTENT(IN) :: ki
        REAL(8), INTENT(IN) :: freq
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8), INTENT(IN) :: DT
        TYPE(RESPARAMS), INTENT(INOUT), TARGET :: resP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8) :: ResController_result
        ResController_result = REAL(rescontroller_c(error, kp, ki, freq, minValue, maxValue, DT, C_LOC(resP), MERGE(1_C_INT, 0_C_INT, reset), inst), 8)
    END FUNCTION ResController

!-------------------------------------------------------------------------------------------------------------------------------
END MODULE Controllers
