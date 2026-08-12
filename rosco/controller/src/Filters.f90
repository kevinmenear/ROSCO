! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! This module contains all the filters and related subroutines

! Filters:
!       LPFilter: Low-pass filter
!       SecLPFilter: Second order low-pass filter
!       HPFilter: High-pass filter
!       NotchFilter: Notch filter
!       NotchFilterSlopes: Notch Filter with descending slopes
!       PreFilterMeasuredSignals: Pre-filter signals during each run iteration

MODULE Filters
!...............................................................................................................................
    USE Constants
    USE Functions
    USE ISO_C_BINDING
    IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of HPFilter
    INTERFACE
        FUNCTION hpfilter_c(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='hpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: hpfilter_c
        END FUNCTION hpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of LPFilter
    INTERFACE
        FUNCTION lpfilter_c(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='lpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: lpfilter_c
        END FUNCTION lpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of NotchFilter
    INTERFACE
        FUNCTION notchfilter_c(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='notchfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: omega
            REAL(C_DOUBLE), VALUE :: betaNum
            REAL(C_DOUBLE), VALUE :: betaDen
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: notchfilter_c
        END FUNCTION notchfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of NotchFilterSlopes
    INTERFACE
        FUNCTION notchfilterslopes_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_Moving, Moving, has_InitialValue, InitialValue) BIND(C, NAME='notchfilterslopes_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_Moving
            INTEGER(C_INT), VALUE :: Moving
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: notchfilterslopes_c
        END FUNCTION notchfilterslopes_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of SecLPFilter
    INTERFACE
        FUNCTION seclpfilter_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='seclpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_c
        END FUNCTION seclpfilter_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue) RESULT(LPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: LPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        LPFilter_result = REAL(lpfilter_c(InputSignal, DT, CornerFreq, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION LPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        REAL(8), INTENT(IN) :: Damp
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: SecLPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_result = REAL(seclpfilter_c(InputSignal, DT, CornerFreq, Damp, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION SecLPFilter

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue)
    ! Discrete time Low-Pass Filter (output is velocity) of the form:
    !                               Continuous Time Form:   H(s) = s CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
    !                               Discrete Time From:     H(z) = (b2*z^2 + b1*z + b0) / (a2*z^2 + a1*z + a0)
        USE ROSCO_Types, ONLY : FilterParameters
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 
        REAL(DbKi), INTENT(IN)         :: InputSignal
        REAL(DbKi), INTENT(IN)         :: DT                       ! time step [s]
        REAL(DbKi), INTENT(IN)         :: CornerFreq               ! corner frequency [rad/s]
        REAL(DbKi), INTENT(IN)         :: Damp                     ! Dampening constant
        INTEGER(IntKi), INTENT(IN)      :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER(IntKi), INTENT(INOUT)   :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)          :: reset                    ! Reset the filter to the input signal
        REAL(DbKi), OPTIONAL,  INTENT(IN)          :: InitialValue           ! Value to set when reset 
        
        REAL(DbKi)                          :: InitialValue_           ! Value to set when reset

        ! Defaults
        InitialValue_ = InputSignal
        IF (PRESENT(InitialValue)) InitialValue_ = InitialValue  

        ! Initialization
        IF ((iStatus == 0) .OR. reset )  THEN
            FP%lpfV_OutputSignalLast1(inst)  = InitialValue_
            FP%lpfV_OutputSignalLast2(inst)  = InitialValue_
            FP%lpfV_InputSignalLast1(inst)   = InitialValue_
            FP%lpfV_InputSignalLast2(inst)   = InitialValue_
            
            ! Coefficients
            FP%lpfV_a2(inst) = DT**2.0*CornerFreq**2.0 + 4.0 + 4.0*Damp*CornerFreq*DT
            FP%lpfV_a1(inst) = 2.0*DT**2.0*CornerFreq**2.0 - 8.0
            FP%lpfV_a0(inst) = DT**2.0*CornerFreq**2.0 + 4.0 - 4.0*Damp*CornerFreq*DT
            FP%lpfV_b2(inst) = 2.0*DT*CornerFreq**2.0
            FP%lpfV_b1(inst) = 0.0
            FP%lpfV_b0(inst) = -2.0*DT*CornerFreq**2.0
        ENDIF

        ! Filter
        SecLPFilter_Vel = 1.0/FP%lpfV_a2(inst) * (FP%lpfV_b2(inst)*InputSignal + FP%lpfV_b1(inst)*FP%lpfV_InputSignalLast1(inst) + FP%lpfV_b0(inst)*FP%lpfV_InputSignalLast2(inst) - FP%lpfV_a1(inst)*FP%lpfV_OutputSignalLast1(inst) - FP%lpfV_a0(inst)*FP%lpfV_OutputSignalLast2(inst))

        ! Save signals for next time step
        FP%lpfV_InputSignalLast2(inst)   = FP%lpfV_InputSignalLast1(inst)
        FP%lpfV_InputSignalLast1(inst)   = InputSignal
        FP%lpfV_OutputSignalLast2(inst)  = FP%lpfV_OutputSignalLast1(inst)
        FP%lpfV_OutputSignalLast1(inst)  = SecLPFilter_Vel

        inst = inst + 1

    END FUNCTION SecLPFilter_Vel

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION HPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue) RESULT(HPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: HPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        HPFilter_result = REAL(hpfilter_c(InputSignal, DT, CornerFreq, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION HPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, Moving, InitialValue) RESULT(NotchFilterSlopes_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        REAL(8), INTENT(IN) :: Damp
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        LOGICAL, INTENT(IN), OPTIONAL :: Moving
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: NotchFilterSlopes_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_Moving_flag
        INTEGER(C_INT) :: Moving_val
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_Moving_flag = 0
        Moving_val = 0
        IF (PRESENT(Moving)) THEN
            has_Moving_flag = 1
            Moving_val = MERGE(1_C_INT, 0_C_INT, Moving)
        END IF
        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        NotchFilterSlopes_result = REAL(notchfilterslopes_c(InputSignal, DT, CornerFreq, Damp, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_Moving_flag, Moving_val, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION NotchFilterSlopes
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, InitialValue) RESULT(NotchFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: omega
        REAL(8), INTENT(IN) :: betaNum
        REAL(8), INTENT(IN) :: betaDen
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: NotchFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        NotchFilter_result = REAL(notchfilter_c(InputSignal, DT, omega, betaNum, betaDen, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION NotchFilter
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE PreFilterMeasuredSignals(CntrPar, LocalVar, DebugVar, objInst, ErrVar)
    ! Prefilter shared measured wind turbine signals

        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances, ErrorVariables
        
        TYPE(ControlParameters), INTENT(INOUT)      :: CntrPar
        TYPE(LocalVariables),    INTENT(INOUT)      :: LocalVar
        TYPE(DebugVariables),    INTENT(INOUT)      :: DebugVar
        TYPE(ObjectInstances),   INTENT(INOUT)      :: objInst
        TYPE(ErrorVariables),   INTENT(INOUT)       :: ErrVar
        INTEGER(IntKi) :: K  ! Integer used to loop through turbine blades
        INTEGER(IntKi) :: n  ! Integer used to loop through notch filters
        REAL(DbKi)       :: NacVaneCosF                 ! Time-filtered x-component of NacVane (deg)
        REAL(DbKi)       :: NacVaneSinF                 ! Time-filtered y-component of NacVane (deg)

        ! If there's an error, don't even try to run
        IF (ErrVar%aviFAIL < 0) THEN
            RETURN
        ENDIF
        ! Filter the HSS (generator) and LSS (rotor) speed measurement:
        ! Apply Low-Pass Filter (choice between first- and second-order low-pass filter)
        IF (CntrPar%F_LPFType == 1) THEN
            LocalVar%GenSpeedF = LPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
            LocalVar%RotSpeedF = LPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
        ELSEIF (CntrPar%F_LPFType == 2) THEN   
            LocalVar%GenSpeedF = SecLPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, CntrPar%F_LPFDamping, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Second-order low-pass filter on generator speed
            LocalVar%RotSpeedF = SecLPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, CntrPar%F_LPFDamping, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Second-order low-pass filter on generator speed
        ENDIF
        
        ! Apply Notch Fitler to Gen Speed 
        DO n = 1,CntrPar%F_GenSpdNotch_N
            LocalVar%GenSpeedF = NotchFilter(LocalVar%GenSpeedF, LocalVar%DT, &
                                            CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
        END DO

        ! Filtering the tower fore-aft acceleration signal 
        ! Force to start at 0
        IF (LocalVar%iStatus == 0 .AND. LocalVar%Time == 0) THEN
            LocalVar%NacIMU_FA_RAcc = 0
            LocalVar%FA_Acc_Nac = 0
        ENDIF 

        ! Low pass
        LocalVar%NacIMU_FA_AccF = SecLPFilter(LocalVar%NacIMU_FA_RAcc, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Fixed Damping
        LocalVar%FA_AccF = SecLPFilter(LocalVar%FA_Acc_Nac, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Fixed Damping
        
        ! High pass
        LocalVar%NacIMU_FA_AccF = HPFilter(LocalVar%NacIMU_FA_AccF, LocalVar%DT, CntrPar%F_FlHighPassFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instHPF) 
        LocalVar%FA_AccF = HPFilter(LocalVar%FA_AccF, LocalVar%DT, CntrPar%F_FlHighPassFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instHPF) 
        
        ! Notch filters
        DO n = 1,CntrPar%F_TwrTopNotch_N
            LocalVar%NACIMU_FA_AccF = NotchFilter(LocalVar%NacIMU_FA_AccF, LocalVar%DT, &
                                                  CntrPar%F_NotchFreqs(CntrPar%F_TwrTopNotch_Ind(n)), &
                                                  CntrPar%F_NotchBetaNum(CntrPar%F_TwrTopNotch_Ind(n)), &
                                                  CntrPar%F_NotchBetaDen(CntrPar%F_TwrTopNotch_Ind(n)), &
                                                  LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
            
            LocalVar%FA_AccF = NotchFilter(LocalVar%FA_AccF, LocalVar%DT, &
                                           CntrPar%F_NotchFreqs(CntrPar%F_TwrTopNotch_Ind(n)), &
                                           CntrPar%F_NotchBetaNum(CntrPar%F_TwrTopNotch_Ind(n)), &
                                           CntrPar%F_NotchBetaDen(CntrPar%F_TwrTopNotch_Ind(n)), &
                                           LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
        END DO
        
        ! FA acc for ForeAft damping, condition matches whether it's used in Controllers.f90
        IF (CntrPar%TD_Mode > 0) THEN
            LocalVar%FA_AccHPF = HPFilter(LocalVar%FA_Acc_Nac, LocalVar%DT, CntrPar%FA_HPFCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instHPF)
        ENDIF
        
        ! Filter Wind Speed Estimator Signal
        LocalVar%We_Vw_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%F_WECornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 

        ! Blade root bending moment for IPC
        DO K = 1,LocalVar%NumBl
            IF ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Flp_Mode == 3)) THEN
                ! Moving inverted notch at rotor speed to isolate 1P
                LocalVar%RootMOOPF(K) = NotchFilterSlopes(LocalVar%rootMOOP(K), LocalVar%DT, LocalVar%RotSpeedF, 0.7_DbKi, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotchSlopes, .TRUE.)
            ELSEIF ( CntrPar%Flp_Mode == 2 ) THEN
                ! Filter Blade root bending moments
                LocalVar%RootMOOPF(K) = SecLPFilter(LocalVar%rootMOOP(K),LocalVar%DT, CntrPar%F_FlpCornerFreq(1), CntrPar%F_FlpCornerFreq(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart,objInst%instSecLPF)
                LocalVar%RootMOOPF(K) = HPFilter(LocalVar%rootMOOPF(K),LocalVar%DT, 0.1_DbKi, LocalVar%FP, LocalVar%iStatus, LocalVar%restart,objInst%instHPF)
                
                ! Apply gen speed notch filters to blade root signal (same as gen speed path)
                DO n = 1,CntrPar%F_GenSpdNotch_N  ! upstream bug: n was uninitialized here
                    LocalVar%RootMOOPF(K) = NotchFilter(LocalVar%RootMOOPF(K), LocalVar%DT, &
                                                        CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
                END DO
            ELSE
                LocalVar%RootMOOPF(K) = LocalVar%rootMOOP(K)
            ENDIF     
        END DO


        ! Control commands (used by WSE, mostly)
        LocalVar%VS_LastGenTrqF = SecLPFilter(LocalVar%VS_LastGenTrq, LocalVar%DT, CntrPar%F_LPFCornerFreq, 0.7_DbKi, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF)
        LocalVar%BlPitchCMeasF    = SecLPFilter(LocalVar%BlPitchCMeas, LocalVar%DT, CntrPar%F_LPFCornerFreq*0.25, 0.7_DbKi, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF)

        ! Wind vane signal
        NacVaneCosF = LPFilter(cos(LocalVar%NacVane*D2R), LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
        NacVaneSinF = LPFilter(sin(LocalVar%NacVane*D2R), LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
        LocalVar%NacVaneF = wrap_180(atan2(NacVaneSinF, NacVaneCosF) * R2D) ! (deg)

        ! Debug Variables
        DebugVar%GenSpeedF = LocalVar%GenSpeedF
        DebugVar%RotSpeedF = LocalVar%RotSpeedF
        DebugVar%NacIMU_FA_AccF = LocalVar%NacIMU_FA_AccF
        DebugVar%FA_AccF = LocalVar%FA_AccF
    END SUBROUTINE PreFilterMeasuredSignals
    END MODULE Filters
