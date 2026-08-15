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

USE, INTRINSIC :: ISO_C_Binding
USE Constants
USE Filters
USE Functions
USE SysSubs

IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of StateMachine
    INTERFACE
        SUBROUTINE statemachine_c(CntrPar, LocalVar) BIND(C, NAME='statemachine_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
        END SUBROUTINE statemachine_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PitchSaturation
    INTERFACE
        FUNCTION pitchsaturation_c(LocalVar, CntrPar, objInst, DebugVar, ErrVar) BIND(C, NAME='pitchsaturation_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: pitchsaturation_c
        END FUNCTION pitchsaturation_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PowerControlSetpoints
    INTERFACE
        SUBROUTINE powercontrolsetpoints_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='powercontrolsetpoints_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE powercontrolsetpoints_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of SetpointSmoother
    INTERFACE
        SUBROUTINE setpointsmoother_c(LocalVar, CntrPar, objInst) BIND(C, NAME='setpointsmoother_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE setpointsmoother_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Shutdown
    INTERFACE
        SUBROUTINE shutdown_c(LocalVar, CntrPar, objInst, ErrVar) BIND(C, NAME='shutdown_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE shutdown_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Startup
    INTERFACE
        SUBROUTINE startup_c(LocalVar, CntrPar, objInst, ErrVar) BIND(C, NAME='startup_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE startup_c
    END INTERFACE

CONTAINS

    SUBROUTINE PowerControlSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
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
        CALL powercontrolsetpoints_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE PowerControlSetpoints

! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE Constants
        ! Allocate variables
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar


        !   Change pitch reference speed
        LocalVar%PC_RefSpd_PRC = CntrPar%PC_RefSpd * LocalVar%PRC_R_Speed
        
        ! Lookup table for speed setpoint (PRC_Mode 1)
        IF (CntrPar%PRC_Mode == 1) THEN
            LocalVar%PRC_WSE_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%PRC_LPF_Freq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 
            LocalVar%PC_RefSpd_PRC = interp1d(CntrPar%PRC_WindSpeeds,CntrPar%PRC_GenSpeeds,LocalVar%PRC_WSE_F,ErrVar)
        ENDIF

        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF < 0) THEN
            LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC - LocalVar%SS_DelOmegaF
        ELSE
            LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC
        ENDIF

        ! Compute error for pitch controller
        LocalVar%PC_RefSpd = LocalVar%PC_RefSpd_SS        
        LocalVar%PC_SpdErr = LocalVar%PC_RefSpd - LocalVar%GenSpeedF            ! Speed error
        LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr             ! Power error, unused

        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]
        IF (CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) THEN
            ! Use unfiltered wind speed estimate, then filter below
            LocalVar%VS_RefSpd_TSR = (CntrPar%VS_TSRopt * LocalVar%WE_Vw / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio

        ELSEIF (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) THEN ! Genspeed reference that doesn't depend on wind speed estimate (https://doi.org/10.2172/1259805)
            LocalVar%VS_RefSpd_TSR = (MAX(LocalVar%VS_GenPwr, 0.0)/(CntrPar%VS_GenEff/100.0)/CntrPar%VS_Rgn2K)**(1./3.)

        ELSEIF (CntrPar%VS_ControlMode == VS_Mode_Torque_TSR) THEN ! Non-WSE TSR tracking based on square-root of torque
            LocalVar%VS_RefSpd_TSR = (MAX(LocalVar%GenTq, 0.0)/CntrPar%VS_Rgn2K)**(1./2.)

        ELSE ! Generate constant speed reference if K*Omega^2 in use or torque control disabled
            LocalVar%VS_RefSpd_TSR = CntrPar%VS_RefSpd
        ENDIF 

        ! Region 3 FBP reference logic, triggers if Region-2 reference speed is higher than rated
        ! DBS: Alternatively, each of these alternative reference modes could identify Region 3 using their reference-deriving signal, e.g. if WE_Vw > rated speed (accessible in ROSCO?) or GenTq > VS_RtTq
        IF (LocalVar%VS_RefSpd_TSR > CntrPar%VS_RefSpd) THEN
            IF (CntrPar%VS_FBP == VS_FBP_WSE_Ref) THEN ! Use WSE to look up speed reference in Region 3
                LocalVar%VS_RefSpd_TSR = interp1d(CntrPar%VS_FBP_U, CntrPar%VS_FBP_Omega, LocalVar%WE_Vw, ErrVar)

            ELSEIF (CntrPar%VS_FBP == VS_FBP_Torque_Ref) THEN ! Use torque to look up speed reference in Region 3
                LocalVar%VS_RefSpd_TSR = interp1d(CntrPar%VS_FBP_Tau, CntrPar%VS_FBP_Omega, LocalVar%GenTq, ErrVar)

            ENDIF
        ENDIF

        ! Change VS Ref speed based on R_Speed
        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_TSR * LocalVar%PRC_R_Speed


        ! Filter reference signal
        LocalVar%VS_RefSpd = LPFilter(LocalVar%VS_RefSpd_TSR, LocalVar%DT, CntrPar%F_VSRefSpdCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)


        ! Exclude reference speeds specified by user
        IF (CntrPar%TRA_Mode > 0) THEN
            CALL RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
        END IF

        ! Saturate torque reference speed below rated speed if using pitch control in Region 3
        IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
            LocalVar%VS_RefSpd = saturate(LocalVar%VS_RefSpd, CntrPar%VS_MinOMSpd, CntrPar%VS_RefSpd * LocalVar%PRC_R_Speed)
        END IF

        ! Simple lookup table for generator speed (PRC_Mode 1)
        IF (CntrPar%PRC_Mode == 1) THEN
            LocalVar%VS_RefSpd = interp1d(CntrPar%PRC_WindSpeeds,CntrPar%PRC_GenSpeeds,LocalVar%PRC_WSE_F,ErrVar)
        ENDIF

        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF > 0) THEN
            LocalVar%VS_RefSpd = LocalVar%VS_RefSpd - LocalVar%SS_DelOmegaF
        ENDIF

        ! Force minimum rotor speed
        LocalVar%VS_RefSpd = max(LocalVar%VS_RefSpd, CntrPar%VS_MinOmSpd)

        ! Compute speed error from reference
        LocalVar%VS_SpdErr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF

        ! Define transition region setpoint errors
        LocalVar%VS_SpdErrAr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF               ! Current speed error - Region 2.5 PI-control (Above Rated)
        LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF     ! Current speed error - Region 1.5 PI-control (Below Rated)
        
        ! Region 3 minimum pitch angle for state machine
        LocalVar%VS_Rgn3Pitch = LocalVar%PC_MinPit + CntrPar%PC_Switch

        ! Debug Vars
        DebugVar%VS_RefSpd = LocalVar%VS_RefSpd
        DebugVar%PC_RefSpd = LocalVar%PC_RefSpd


    END SUBROUTINE ComputeVariablesSetpoints
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StateMachine(CntrPar, LocalVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL statemachine_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    ! Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
    !       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
    !       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, DebugVariables, ErrorVariables
        USE ieee_arithmetic
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)       :: PerfData
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        ! Allocate Variables
        REAL(DbKi)                 :: F_WECornerFreq   ! Corner frequency (-3dB point) for first order low pass filter for measured hub height wind speed [Hz]

        !       Only used in EKF, if WE_Mode = 2
        REAL(DbKi)                 :: L                ! Turbulent length scale parameter [m]
        REAL(DbKi)                 :: Ti               ! Turbulent intensity, [-]
        ! REAL(DbKi), DIMENSION(3,3) :: I
        !           - operating conditions
        REAL(DbKi)                 :: A_op             ! Estimated operational system pole [UNITS!]
        REAL(DbKi)                 :: Cp_op            ! Estimated operational Cp [-]
        REAL(DbKi)                 :: Tau_r            ! Estimated rotor torque [Nm]
        REAL(DbKi)                 :: a                ! wind variance
        REAL(DbKi)                 :: lambda           ! tip-speed-ratio [rad]
        REAL(DbKi)                 :: RotSpeed         ! Rotor Speed [rad], locally

        !           - Covariance matrices
        REAL(DbKi), DIMENSION(3,3)         :: F        ! First order system jacobian 
        REAL(DbKi), DIMENSION(1,3)         :: H        ! Output equation jacobian 
        REAL(DbKi), DIMENSION(3,1)         :: dxh      ! Estimated state matrix deviation from previous timestep
        REAL(DbKi), DIMENSION(3,3)         :: Q        ! Process noise covariance matrix
        REAL(DbKi), DIMENSION(1,1)         :: S        ! Innovation covariance 
        REAL(DbKi)                         :: R_m      ! Measurement noise covariance [(rad/s)^2]

        REAL(DbKi)              :: WE_Inp_Pitch
        REAL(DbKi)              :: WE_Inp_Torque
        REAL(DbKi)              :: WE_Inp_Speed
        REAL(DbKi)              :: Max_Op_Pitch
        
        CHARACTER(*), PARAMETER                 :: RoutineName = 'WindSpeedEstimator'
        CHARACTER(1024)                        :: WarningMessage

        

        ! Saturate inputs to WSE:
        ! Rotor speed
        IF (LocalVar%RotSpeedF < 0.25 * CntrPar%VS_MinOMSpd / CntrPar%WE_GearboxRatio) THEN
            WE_Inp_Speed = 0.25 * CntrPar%VS_MinOMSpd / CntrPar%WE_GearboxRatio + EPSILON(1.0_DbKi)  ! If this is 0, could cause problems...
        ELSE
            WE_Inp_Speed = LocalVar%RotSpeedF
        END IF

            
        ! Blade pitch
        IF (CntrPar%WE_Mode > 0) THEN ! PerfData is only loaded if WE_Mode > 0
            Max_Op_Pitch = PerfData%Beta_vec(SIZE(PerfData%Beta_vec)) * D2R     ! The Cp surface is only valid up to the end of Beta_vec
        ELSE
            Max_Op_Pitch = 0.0_DbKi    ! Doesn't matter if WE_Mode = 0
        ENDIF
        
        WE_Inp_Pitch = saturate(LocalVar%BlPitchCMeas, CntrPar%PC_MinPit,Max_Op_Pitch) 


        ! Gen torque
        IF (LocalVar%VS_LastGenTrqF < 0.0001 * CntrPar%VS_RtTq) THEN
            WE_Inp_Torque = 0.0001 * CntrPar%VS_RtTq
        ELSE
            WE_Inp_Torque = LocalVar%VS_LastGenTrqF
        END IF

        ! Check to see if in operational range
        LocalVar%WE_Op_Last = LocalVar%WE_Op
        IF (ABS(WE_Inp_Pitch - LocalVar%BlPitchCMeas) > 0) THEN
            LocalVar%WE_Op = 0
        ELSEIF (ABS(WE_Inp_Torque - LocalVar%VS_LastGenTrqF) > 0) THEN
            LocalVar%WE_Op = 0
        ELSEIF (ABS(WE_Inp_Speed - LocalVar%RotSpeedF) > 0) THEN
            LocalVar%WE_Op = 0
        ELSE
            LocalVar%WE_Op = 1
        ENDIF


        ! Restart flag for WSE
        LocalVar%RestartWSE = LocalVar%iStatus      ! Same as iStatus by default

        IF (CntrPar%WE_Mode > 0) THEN
            IF (LocalVar%WE_Op == 0 .AND. LocalVar%WE_Op_Last == 1) THEN   ! Transition from operational to non-operational
                WarningMessage = NewLine//'***************************************************************************************************************************************'//NewLine// &
                    'ROSCO Warning: The wind speed estimator is used, but an input (pitch, rotor speed, or torque) has left the bounds of normal operation.'//NewLine// &
                    'The filtered hub-height wind speed will be used instead. This warning will not persist even though the condition may.'//NewLine// &
                    'Check WE_Op in the ROSCO .dbg file to see if the WSE is enabled (1) or disabled (0).'//NewLine// &
                    '***************************************************************************************************************************************'
                PRINT *, TRIM(WarningMessage)

                LocalVar%RestartWSE = 0 ! Restart
            ENDIF

            IF (LocalVar%WE_Op == 1 .AND. LocalVar%WE_Op_Last == 0) THEN    ! Transition from non-operational to operational
                LocalVar%RestartWSE = 0  ! Restart
            ENDIF
        ENDIF

        ! Filter the wind speed at hub height regardless, only use if WE_Mode = 0 or WE_Op = 0
        ! Re-initialize at WE_Vw if leaving operational wind, WE_Vw is initialized at HorWindV
        LocalVar%HorWindV_F = cos(LocalVar%NacVaneF*D2R) * LPFilter(LocalVar%HorWindV, LocalVar%DT, CntrPar%F_WECornerFreq/10, LocalVar%FP, LocalVar%RestartWSE, LocalVar%restart, objInst%instLPF, LocalVar%WE_Vw)

        ! ---- Debug Inputs ------
        DebugVar%WE_b   = WE_Inp_Pitch
        DebugVar%WE_w   = WE_Inp_Speed
        DebugVar%WE_t   = WE_Inp_Torque

        ! ---- Define wind speed estimate ---- 
        
        ! Inversion and Invariance Filter implementation
        IF (CntrPar%WE_Mode == 1 .AND. LocalVar%WE_Op > 0) THEN      
            ! Compute AeroDynTorque
            Tau_r = AeroDynTorque(LocalVar%RotSpeedF, LocalVar%BlPitchCMeas, LocalVar, CntrPar, PerfData, ErrVar)

            LocalVar%WE_VwIdot = CntrPar%WE_Gamma/CntrPar%WE_Jtot*(LocalVar%VS_LastGenTrq*CntrPar%WE_GearboxRatio - Tau_r)
            LocalVar%WE_VwI = LocalVar%WE_VwI + LocalVar%WE_VwIdot*LocalVar%DT
            LocalVar%WE_Vw = LocalVar%WE_VwI + CntrPar%WE_Gamma*LocalVar%RotSpeedF

        ! Extended Kalman Filter (EKF) implementation
        ELSEIF (CntrPar%WE_Mode == 2 .AND. LocalVar%WE_Op > 0) THEN
            ! Define contant values
            L = 6.0 * CntrPar%WE_BladeRadius
            Ti = 0.18
            R_m = 0.02
            H = RESHAPE((/1.0 , 0.0 , 0.0/),(/1,3/))
            ! Define matrices to be filled
            F = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            Q = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            IF (LocalVar%RestartWSE == 0) THEN
                ! Initialize recurring values
                LocalVar%WE%om_r = WE_Inp_Speed
                LocalVar%WE%v_t = 0.0
                LocalVar%WE%v_m = max(LocalVar%HorWindV_F, 3.0_DbKi)   ! avoid divide by 0 below if HorWindV_F is 0, which some AMRWind setups create
                LocalVar%WE%v_h = max(LocalVar%HorWindV_F, 3.0_DbKi)   ! avoid divide by 0 below if HorWindV_F is 0, which some AMRWind setups create
                LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t   ! Initialize WE_Vw to aviod divide by zero
                lambda = WE_Inp_Speed * CntrPar%WE_BladeRadius/LocalVar%WE%v_h
                LocalVar%WE%xh = RESHAPE((/LocalVar%WE%om_r, LocalVar%WE%v_t, LocalVar%WE%v_m/),(/3,1/))
                LocalVar%WE%P = RESHAPE((/0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0/),(/3,3/))
                LocalVar%WE%K = RESHAPE((/0.0,0.0,0.0/),(/3,1/))
                Cp_op   = 0.25  ! initialize so debug output doesn't give *****
                
            ELSE

                ! Find estimated operating Cp and system pole
                A_op = interp1d(CntrPar%WE_FOPoles_v,CntrPar%WE_FOPoles,LocalVar%WE%v_h,ErrVar)

                ! TEST INTERP2D
                lambda = max(WE_Inp_Speed, EPSILON(1.0_DbKi)) * CntrPar%WE_BladeRadius/LocalVar%WE%v_h
                Cp_op = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, WE_Inp_Pitch*R2D, lambda , ErrVar)
                Cp_op = max(0.0,Cp_op)
                
                ! Update Jacobian
                F(1,1) = A_op
                F(1,2) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * Cp_op * LocalVar%WE%v_h**2.0
                F(1,3) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * Cp_op * LocalVar%WE%v_h**2.0
                F(2,2) = - PI * LocalVar%WE%v_m/(2.0*L)
                F(2,3) = - PI * LocalVar%WE%v_t/(2.0*L)
                
                ! Update process noise covariance
                Q(1,1) = 0.00001
                Q(2,2) =(PI * (LocalVar%WE%v_m**3.0) * (Ti**2.0)) / L
                Q(3,3) = (2.0**2.0)/600.0
                
                ! Prediction update
                Tau_r = AeroDynTorque(WE_Inp_Speed, WE_Inp_Pitch, LocalVar, CntrPar, PerfData, ErrVar)
                a = PI * LocalVar%WE%v_m/(2.0*L)
                dxh(1,1) = 1.0/CntrPar%WE_Jtot * (Tau_r - CntrPar%WE_GearboxRatio * WE_Inp_Torque)
                dxh(2,1) = -a*LocalVar%WE%v_t
                dxh(3,1) = 0.0
                
                LocalVar%WE%xh = LocalVar%WE%xh + LocalVar%DT * dxh ! state update
                LocalVar%WE%P = LocalVar%WE%P + LocalVar%DT*(MATMUL(F,LocalVar%WE%P) + MATMUL(LocalVar%WE%P,TRANSPOSE(F)) + Q - MATMUL(LocalVar%WE%K * R_m, TRANSPOSE(LocalVar%WE%K))) 
                
                ! Measurement update
                S = MATMUL(H,MATMUL(LocalVar%WE%P,TRANSPOSE(H))) + R_m        ! NJA: (H*T*H') \approx 0
                LocalVar%WE%K = MATMUL(LocalVar%WE%P,TRANSPOSE(H))/S(1,1)
                LocalVar%WE%xh = LocalVar%WE%xh + LocalVar%WE%K*(WE_Inp_Speed - LocalVar%WE%om_r)
                LocalVar%WE%P = MATMUL(identity(3) - MATMUL(LocalVar%WE%K,H),LocalVar%WE%P)
                
                
                ! Wind Speed Estimate
                LocalVar%WE%om_r = max(LocalVar%WE%xh(1,1), EPSILON(1.0_DbKi))
                LocalVar%WE%v_t = LocalVar%WE%xh(2,1)
                LocalVar%WE%v_m = LocalVar%WE%xh(3,1)
                LocalVar%WE%v_h = LocalVar%WE%v_t + LocalVar%WE%v_m
                LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t

                IF (ieee_is_nan(LocalVar%WE%v_h)) THEN
                    LocalVar%WE%om_r = WE_Inp_Speed
                    LocalVar%WE%v_t = 0.0
                    LocalVar%WE%v_m = LocalVar%HorWindV
                    LocalVar%WE%v_h = LocalVar%HorWindV
                    LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t
                ENDIF

            ENDIF
            ! Debug Outputs
            DebugVar%WE_Cp = Cp_op
            DebugVar%WE_Vm = LocalVar%WE%v_m
            DebugVar%WE_Vt = LocalVar%WE%v_t
            DebugVar%WE_lambda = lambda
        ELSE        
            ! Use filtered hub-height
            LocalVar%WE_Vw = LocalVar%HorWindV_F
        ENDIF 
        DebugVar%WE_Vw = LocalVar%WE_Vw
        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL setpointsmoother_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar) RESULT(PitchSaturation_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: PitchSaturation_result
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        PitchSaturation_result = REAL(pitchsaturation_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION PitchSaturation
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Startup(LocalVar, CntrPar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL startup_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE Startup
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Shutdown(LocalVar, CntrPar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL shutdown_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE Shutdown
!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar) 
    ! Reference speed exclusion:
    !   Changes torque controllerr reference speed to avoid specified frequencies by a prescribed bandwidth
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(DebugVariables),      INTENT(INOUT)        :: DebugVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst

        
        REAL(DbKi)                             :: VS_RefSpeed_LSS
        
        ! Get LSS Ref speed
        VS_RefSpeed_LSS = LocalVar%VS_RefSpd/CntrPar%WE_GearboxRatio

        IF ((VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2) .AND. &
            (VS_RefSpeed_LSS < CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2)) THEN
            ! In hysteresis zone, hold reference speed
            LocalVar%FA_Hist = 1 ! Set negative hysteris if ref < exclusion band
        ELSE
            LocalVar%FA_Hist = 0
        END IF

        ! Initialize last reference speed state
        IF (LocalVar%restart) THEN
            ! If starting in hist band
            IF (LocalVar%FA_Hist > 0) THEN
                IF (VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed) THEN
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2
                ELSE
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2
                ENDIF
            ELSE
                LocalVar%TRA_LastRefSpd = VS_RefSpeed_LSS
            END IF
        END IF 


        IF (LocalVar%FA_Hist > 0) THEN
            LocalVar%VS_RefSpd_TRA = LocalVar%TRA_LastRefSpd
        ELSE
            LocalVar%VS_RefSpd_TRA = VS_RefSpeed_LSS
        END IF

        ! Save last reference speed       
        LocalVar%TRA_LastRefSpd = LocalVar%VS_RefSpd_TRA

        ! Rate limit reference speed
        LocalVar%VS_RefSpd_RL = ratelimit(LocalVar%VS_RefSpd_TRA, -CntrPar%TRA_RateLimit, CntrPar%TRA_RateLimit, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL)
        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_RL * CntrPar%WE_GearboxRatio


        
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks
