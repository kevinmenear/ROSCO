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


    ! Auto-generated interface for C++ implementation of LPFilter
    INTERFACE
        FUNCTION lpfilter_c(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='lpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: lpfilter_c
        END FUNCTION lpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of HPFilter
    INTERFACE
        FUNCTION hpfilter_c(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='hpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: hpfilter_c
        END FUNCTION hpfilter_c
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
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_c
        END FUNCTION seclpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of SecLPFilter_Vel
    INTERFACE
        FUNCTION seclpfilter_vel_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='seclpfilter_vel_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_vel_c
        END FUNCTION seclpfilter_vel_c
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
            LOGICAL(C_BOOL), VALUE :: reset
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
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_Moving
            LOGICAL(C_BOOL), VALUE :: Moving
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: notchfilterslopes_c
        END FUNCTION notchfilterslopes_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue) RESULT(LPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: LPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        LPFilter_result = REAL(lpfilter_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION LPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        REAL(DbKi), INTENT(IN) :: Damp
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: SecLPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_result = REAL(seclpfilter_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), REAL(Damp, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION SecLPFilter

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_Vel_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        REAL(DbKi), INTENT(IN) :: Damp
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: SecLPFilter_Vel_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_Vel_result = REAL(seclpfilter_vel_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), REAL(Damp, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION SecLPFilter_Vel

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION HPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue) RESULT(HPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: HPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        HPFilter_result = REAL(hpfilter_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION HPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, Moving, InitialValue) RESULT(NotchFilterSlopes_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        REAL(DbKi), INTENT(IN) :: Damp
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        LOGICAL, INTENT(IN), OPTIONAL :: Moving
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: NotchFilterSlopes_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_Moving_flag
        LOGICAL(C_BOOL) :: Moving_val
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_Moving_flag = 0
        Moving_val = 0
        IF (PRESENT(Moving)) THEN
            has_Moving_flag = 1
            Moving_val = Moving
        END IF
        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        NotchFilterSlopes_result = REAL(notchfilterslopes_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), REAL(Damp, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_Moving_flag, Moving_val, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION NotchFilterSlopes
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, InitialValue) RESULT(NotchFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: omega
        REAL(DbKi), INTENT(IN) :: betaNum
        REAL(DbKi), INTENT(IN) :: betaDen
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: NotchFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        NotchFilter_result = REAL(notchfilter_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(omega, C_DOUBLE), REAL(betaNum, C_DOUBLE), REAL(betaDen, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
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
            LocalVar%GenSpeedF = LPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instLPF)
            LocalVar%RotSpeedF = LPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instLPF)
        ELSEIF (CntrPar%F_LPFType == 2) THEN   
            LocalVar%GenSpeedF = SecLPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, CntrPar%F_LPFDamping, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF) ! Second-order low-pass filter on generator speed
            LocalVar%RotSpeedF = SecLPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, CntrPar%F_LPFDamping, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF) ! Second-order low-pass filter on generator speed
        ENDIF
        
        ! Apply Notch Fitler to Gen Speed 
        DO n = 1,CntrPar%F_GenSpdNotch_N
            LocalVar%GenSpeedF = NotchFilter(LocalVar%GenSpeedF, LocalVar%DT, &
                                            CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instNotch)
        END DO

        ! Filtering the tower fore-aft acceleration signal 
        ! Force to start at 0
        IF (LocalVar%iStatus == 0 .AND. LocalVar%Time == 0) THEN
            LocalVar%NacIMU_FA_RAcc = 0
            LocalVar%FA_Acc_Nac = 0
        ENDIF 

        ! Low pass
        LocalVar%NacIMU_FA_AccF = SecLPFilter(LocalVar%NacIMU_FA_RAcc, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF) ! Fixed Damping
        LocalVar%FA_AccF = SecLPFilter(LocalVar%FA_Acc_Nac, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF) ! Fixed Damping
        
        ! High pass
        LocalVar%NacIMU_FA_AccF = HPFilter(LocalVar%NacIMU_FA_AccF, LocalVar%DT, CntrPar%F_FlHighPassFreq, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instHPF) 
        LocalVar%FA_AccF = HPFilter(LocalVar%FA_AccF, LocalVar%DT, CntrPar%F_FlHighPassFreq, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instHPF) 
        
        ! Notch filters
        DO n = 1,CntrPar%F_TwrTopNotch_N
            LocalVar%NACIMU_FA_AccF = NotchFilter(LocalVar%NacIMU_FA_AccF, LocalVar%DT, &
                                                  CntrPar%F_NotchFreqs(CntrPar%F_TwrTopNotch_Ind(n)), &
                                                  CntrPar%F_NotchBetaNum(CntrPar%F_TwrTopNotch_Ind(n)), &
                                                  CntrPar%F_NotchBetaDen(CntrPar%F_TwrTopNotch_Ind(n)), &
                                                  LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instNotch)
            
            LocalVar%FA_AccF = NotchFilter(LocalVar%FA_AccF, LocalVar%DT, &
                                           CntrPar%F_NotchFreqs(CntrPar%F_TwrTopNotch_Ind(n)), &
                                           CntrPar%F_NotchBetaNum(CntrPar%F_TwrTopNotch_Ind(n)), &
                                           CntrPar%F_NotchBetaDen(CntrPar%F_TwrTopNotch_Ind(n)), &
                                           LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instNotch)
        END DO
        
        ! FA acc for ForeAft damping, condition matches whether it's used in Controllers.f90
        IF (CntrPar%TD_Mode > 0) THEN
            LocalVar%FA_AccHPF = HPFilter(LocalVar%FA_Acc_Nac, LocalVar%DT, CntrPar%FA_HPFCornerFreq, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instHPF)
        ENDIF
        
        ! Filter Wind Speed Estimator Signal
        LocalVar%We_Vw_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%F_WECornerFreq, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instLPF) 

        ! Blade root bending moment for IPC
        DO K = 1,LocalVar%NumBl
            IF ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Flp_Mode == 3)) THEN
                ! Moving inverted notch at rotor speed to isolate 1P
                LocalVar%RootMOOPF(K) = NotchFilterSlopes(LocalVar%rootMOOP(K), LocalVar%DT, LocalVar%RotSpeedF, 0.7_DbKi, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instNotchSlopes, .TRUE.)
            ELSEIF ( CntrPar%Flp_Mode == 2 ) THEN
                ! Filter Blade root bending moments
                LocalVar%RootMOOPF(K) = SecLPFilter(LocalVar%rootMOOP(K),LocalVar%DT, CntrPar%F_FlpCornerFreq(1), CntrPar%F_FlpCornerFreq(2), LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0),objInst%instSecLPF)
                LocalVar%RootMOOPF(K) = HPFilter(LocalVar%rootMOOPF(K),LocalVar%DT, 0.1_DbKi, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0),objInst%instHPF)
                
                ! Apply gen speed notch filters to blade root signal (same as gen speed path)
                DO n = 1,CntrPar%F_GenSpdNotch_N  ! VIT fix: n was uninitialized (ROSCO bug)
                    LocalVar%RootMOOPF(K) = NotchFilter(LocalVar%RootMOOPF(K), LocalVar%DT, &
                                                        CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instNotch)
                END DO
            ELSE
                LocalVar%RootMOOPF(K) = LocalVar%rootMOOP(K)
            ENDIF     
        END DO


        ! Control commands (used by WSE, mostly)
        LocalVar%VS_LastGenTrqF = SecLPFilter(LocalVar%VS_LastGenTrq, LocalVar%DT, CntrPar%F_LPFCornerFreq, 0.7_DbKi, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF)
        LocalVar%BlPitchCMeasF    = SecLPFilter(LocalVar%BlPitchCMeas, LocalVar%DT, CntrPar%F_LPFCornerFreq*0.25, 0.7_DbKi, LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF)

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
