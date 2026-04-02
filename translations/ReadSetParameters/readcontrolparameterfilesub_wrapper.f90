    SUBROUTINE ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, accINFILE_size, RootName, ErrVar)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters, ErrorVariables, LocalVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, &
            vit_populate_controlparameters, vit_copy_scalars_to_cntrpar

        IMPLICIT NONE

        INTEGER(IntKi)                                  :: accINFILE_size
        CHARACTER(accINFILE_size),  INTENT(IN   )       :: accINFILE(accINFILE_size)
        TYPE(ControlParameters),    INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT), TARGET :: LocalVar
        TYPE(ErrorVariables),       INTENT(INOUT), TARGET :: ErrVar
        CHARACTER(accINFILE_size),  INTENT(IN)          :: RootName

        ! Local variables for C++ interop
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        CHARACTER(1024)                        :: filename_c
        CHARACTER(1024)                        :: priPath_c
        INTEGER(IntKi)                         :: i
        INTEGER(C_INT32_T)                     :: n_OL_rows, OL_Count

        INTERFACE
            SUBROUTINE readcontrolparameterfilesub_pass1_c(CntrPar, LocalVar, &
                    filename, priPath, ErrVar, n_OL_rows, OL_Count) &
                    BIND(C, NAME='readcontrolparameterfilesub_pass1_c')
                USE ISO_C_BINDING
                TYPE(C_PTR), VALUE :: CntrPar
                TYPE(C_PTR), VALUE :: LocalVar
                CHARACTER(KIND=C_CHAR), INTENT(IN) :: filename(*)
                CHARACTER(KIND=C_CHAR), INTENT(IN) :: priPath(*)
                TYPE(C_PTR), VALUE :: ErrVar
                INTEGER(C_INT32_T), INTENT(OUT) :: n_OL_rows
                INTEGER(C_INT32_T), INTENT(OUT) :: OL_Count
            END SUBROUTINE readcontrolparameterfilesub_pass1_c

            SUBROUTINE readcontrolparameterfilesub_pass2_c(CntrPar, LocalVar, &
                    filename, priPath, ErrVar) &
                    BIND(C, NAME='readcontrolparameterfilesub_pass2_c')
                USE ISO_C_BINDING
                TYPE(C_PTR), VALUE :: CntrPar
                TYPE(C_PTR), VALUE :: LocalVar
                CHARACTER(KIND=C_CHAR), INTENT(IN) :: filename(*)
                CHARACTER(KIND=C_CHAR), INTENT(IN) :: priPath(*)
                TYPE(C_PTR), VALUE :: ErrVar
            END SUBROUTINE readcontrolparameterfilesub_pass2_c
        END INTERFACE

        ! Extract filename from accINFILE (C_CHAR array) and null-terminate
        filename_c = ' '
        DO i = 1, MIN(accINFILE_size, 1023)
            IF (accINFILE(1)(i:i) == C_NULL_CHAR) EXIT
            filename_c(i:i) = accINFILE(1)(i:i)
        END DO
        filename_c(MIN(i, 1024):MIN(i, 1024)) = C_NULL_CHAR

        ! Extract directory path (PriPath) from filename
        priPath_c = ' '
        DO i = MIN(accINFILE_size, 1023), 1, -1
            IF (filename_c(i:i) == '/' .OR. filename_c(i:i) == '\') EXIT
        END DO
        IF (i > 0) THEN
            priPath_c(1:i) = filename_c(1:i)
            priPath_c(i+1:i+1) = C_NULL_CHAR
        ELSE
            priPath_c(1:2) = './'
            priPath_c(3:3) = C_NULL_CHAR
        END IF

        ! ============================================================
        ! PASS 1: Parse all scalars + count OL rows
        ! ============================================================
        n_OL_rows = 0
        OL_Count = 0
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL readcontrolparameterfilesub_pass1_c( &
            C_LOC(CntrPar_view), C_LOC(LocalVar), &
            filename_c, priPath_c, C_LOC(ErrVar), &
            n_OL_rows, OL_Count)
        IF (ErrVar%aviFAIL < 0) RETURN

        ! Copy parsed scalars from view back to CntrPar
        CALL vit_copy_scalars_to_cntrpar(CntrPar_view, CntrPar)

        ! Apply Fl_n default (matches Fortran line 583)
        IF (CntrPar%Fl_n == 0) CntrPar%Fl_n = 1

        ! ============================================================
        ! ALLOCATE all ALLOCATABLE arrays using parsed sizes
        ! ============================================================
        ! Filters
        IF (CntrPar%F_NumNotchFilts > 0) THEN
            ALLOCATE(CntrPar%F_NotchFreqs(CntrPar%F_NumNotchFilts))
            ALLOCATE(CntrPar%F_NotchBetaNum(CntrPar%F_NumNotchFilts))
            ALLOCATE(CntrPar%F_NotchBetaDen(CntrPar%F_NumNotchFilts))
        ELSE
            ALLOCATE(CntrPar%F_NotchFreqs(1)); CntrPar%F_NotchFreqs = 0.0
            ALLOCATE(CntrPar%F_NotchBetaNum(1)); CntrPar%F_NotchBetaNum = 0.0
            ALLOCATE(CntrPar%F_NotchBetaDen(1)); CntrPar%F_NotchBetaDen = 0.0
        END IF
        IF (CntrPar%F_GenSpdNotch_N > 0) ALLOCATE(CntrPar%F_GenSpdNotch_Ind(CntrPar%F_GenSpdNotch_N))
        IF (CntrPar%F_TwrTopNotch_N > 0) ALLOCATE(CntrPar%F_TwrTopNotch_Ind(CntrPar%F_TwrTopNotch_N))
        ALLOCATE(CntrPar%F_FlCornerFreq(2)); CntrPar%F_FlCornerFreq = 0.0
        ALLOCATE(CntrPar%F_FlpCornerFreq(2)); CntrPar%F_FlpCornerFreq = 0.0

        ! Pitch Control
        ALLOCATE(CntrPar%PC_GS_angles(MAX(CntrPar%PC_GS_n, 1))); CntrPar%PC_GS_angles = 0.0
        ALLOCATE(CntrPar%PC_GS_KP(MAX(CntrPar%PC_GS_n, 1))); CntrPar%PC_GS_KP = 0.0
        ALLOCATE(CntrPar%PC_GS_KI(MAX(CntrPar%PC_GS_n, 1))); CntrPar%PC_GS_KI = 0.0
        ALLOCATE(CntrPar%PC_GS_KD(MAX(CntrPar%PC_GS_n, 1))); CntrPar%PC_GS_KD = 0.0
        ALLOCATE(CntrPar%PC_GS_TF(MAX(CntrPar%PC_GS_n, 1))); CntrPar%PC_GS_TF = 0.0

        ! IPC
        ALLOCATE(CntrPar%IPC_Vramp(2)); CntrPar%IPC_Vramp = 0.0
        ALLOCATE(CntrPar%IPC_KP(2)); CntrPar%IPC_KP = 0.0
        ALLOCATE(CntrPar%IPC_KI(2)); CntrPar%IPC_KI = 0.0
        ALLOCATE(CntrPar%IPC_aziOffset(2)); CntrPar%IPC_aziOffset = 0.0

        ! VS Torque
        ALLOCATE(CntrPar%VS_KP(MAX(CntrPar%VS_n, 1))); CntrPar%VS_KP = 0.0
        ALLOCATE(CntrPar%VS_KI(MAX(CntrPar%VS_n, 1))); CntrPar%VS_KI = 0.0
        ALLOCATE(CntrPar%VS_FBP_U(MAX(CntrPar%VS_FBP_n, 1))); CntrPar%VS_FBP_U = 0.0
        ALLOCATE(CntrPar%VS_FBP_Omega(MAX(CntrPar%VS_FBP_n, 1))); CntrPar%VS_FBP_Omega = 0.0
        ALLOCATE(CntrPar%VS_FBP_Tau(MAX(CntrPar%VS_FBP_n, 1))); CntrPar%VS_FBP_Tau = 0.0

        ! PRC
        ALLOCATE(CntrPar%PRC_R_Table(MAX(CntrPar%PRC_Table_n, 1))); CntrPar%PRC_R_Table = 0.0
        ALLOCATE(CntrPar%PRC_Pitch_Table(MAX(CntrPar%PRC_Table_n, 1))); CntrPar%PRC_Pitch_Table = 0.0
        ALLOCATE(CntrPar%PRC_WindSpeeds(MAX(CntrPar%PRC_n, 1))); CntrPar%PRC_WindSpeeds = 0.0
        ALLOCATE(CntrPar%PRC_GenSpeeds(MAX(CntrPar%PRC_n, 1))); CntrPar%PRC_GenSpeeds = 0.0

        ! Wind Speed Estimator
        ALLOCATE(CntrPar%PerfTableSize(2)); CntrPar%PerfTableSize = 0
        ALLOCATE(CntrPar%WE_FOPoles_v(MAX(CntrPar%WE_FOPoles_N, 1))); CntrPar%WE_FOPoles_v = 0.0
        ALLOCATE(CntrPar%WE_FOPoles(MAX(CntrPar%WE_FOPoles_N, 1))); CntrPar%WE_FOPoles = 0.0

        ! Yaw
        ALLOCATE(CntrPar%Y_ErrThresh(2)); CntrPar%Y_ErrThresh = 0.0

        ! Peak Shaving
        ALLOCATE(CntrPar%PS_WindSpeeds(MAX(CntrPar%PS_BldPitchMin_N, 1))); CntrPar%PS_WindSpeeds = 0.0
        ALLOCATE(CntrPar%PS_BldPitchMin(MAX(CntrPar%PS_BldPitchMin_N, 1))); CntrPar%PS_BldPitchMin = 0.0

        ! Startup
        ALLOCATE(CntrPar%SU_LoadStages(MAX(CntrPar%SU_LoadStages_N, 1))); CntrPar%SU_LoadStages = 0.0
        ALLOCATE(CntrPar%SU_LoadRampDuration(MAX(CntrPar%SU_LoadStages_N, 1))); CntrPar%SU_LoadRampDuration = 0.0
        ALLOCATE(CntrPar%SU_LoadHoldDuration(MAX(CntrPar%SU_LoadStages_N, 1))); CntrPar%SU_LoadHoldDuration = 0.0

        ! Shutdown
        ALLOCATE(CntrPar%SD_StageTime(MAX(CntrPar%SD_Stage_N, 1))); CntrPar%SD_StageTime = 0.0
        ALLOCATE(CntrPar%SD_StagePitch(MAX(CntrPar%SD_Stage_N, 1))); CntrPar%SD_StagePitch = 0.0
        ALLOCATE(CntrPar%SD_MaxTorqueRate(MAX(CntrPar%SD_Stage_N, 1))); CntrPar%SD_MaxTorqueRate = 0.0
        ALLOCATE(CntrPar%SD_MaxPitchRate(MAX(CntrPar%SD_Stage_N, 1))); CntrPar%SD_MaxPitchRate = 0.0

        ! Floating
        ALLOCATE(CntrPar%Fl_Kp(CntrPar%Fl_n)); CntrPar%Fl_Kp = 0.0
        ALLOCATE(CntrPar%Fl_U(CntrPar%Fl_n)); CntrPar%Fl_U = 0.0

        ! Open Loop
        ALLOCATE(CntrPar%Ind_BldPitch(3)); CntrPar%Ind_BldPitch = 0
        ALLOCATE(CntrPar%RP_Gains(4)); CntrPar%RP_Gains = 0.0

        ! Pitch Faults
        ALLOCATE(CntrPar%PF_Offsets(3)); CntrPar%PF_Offsets = 0.0
        ALLOCATE(CntrPar%PF_TimeStuck(3)); CntrPar%PF_TimeStuck = 0.0

        ! AWC
        ALLOCATE(CntrPar%AWC_n(MAX(CntrPar%AWC_NumModes, 1))); CntrPar%AWC_n = 0
        ALLOCATE(CntrPar%AWC_harmonic(MAX(CntrPar%AWC_NumModes, 1))); CntrPar%AWC_harmonic = 0
        ALLOCATE(CntrPar%AWC_freq(MAX(CntrPar%AWC_NumModes, 1))); CntrPar%AWC_freq = 0.0
        ALLOCATE(CntrPar%AWC_amp(MAX(CntrPar%AWC_NumModes, 1))); CntrPar%AWC_amp = 0.0
        ALLOCATE(CntrPar%AWC_clockangle(MAX(CntrPar%AWC_NumModes, 1))); CntrPar%AWC_clockangle = 0.0
        ALLOCATE(CntrPar%AWC_CntrGains(2)); CntrPar%AWC_CntrGains = 0.0

        ! Cable / Structural Control
        ALLOCATE(CntrPar%CC_GroupIndex(MAX(CntrPar%CC_Group_N, 1))); CntrPar%CC_GroupIndex = 0
        ALLOCATE(CntrPar%StC_GroupIndex(MAX(CntrPar%StC_Group_N, 1))); CntrPar%StC_GroupIndex = 0
        ALLOCATE(CntrPar%Ind_CableControl(MAX(CntrPar%CC_Group_N, 1))); CntrPar%Ind_CableControl = 0
        ALLOCATE(CntrPar%Ind_StructControl(MAX(CntrPar%StC_Group_N, 1))); CntrPar%Ind_StructControl = 0

        ! Open Loop arrays (size depends on OL file rows)
        IF (CntrPar%OL_Mode > 0 .AND. n_OL_rows > 0) THEN
            ALLOCATE(CntrPar%OL_Channels(n_OL_rows, OL_Count)); CntrPar%OL_Channels = 0.0
            ALLOCATE(CntrPar%OL_Breakpoints(n_OL_rows)); CntrPar%OL_Breakpoints = 0.0
            ALLOCATE(CntrPar%OL_BldPitch1(n_OL_rows)); CntrPar%OL_BldPitch1 = 0.0
            ALLOCATE(CntrPar%OL_BldPitch2(n_OL_rows)); CntrPar%OL_BldPitch2 = 0.0
            ALLOCATE(CntrPar%OL_BldPitch3(n_OL_rows)); CntrPar%OL_BldPitch3 = 0.0
            ALLOCATE(CntrPar%OL_GenTq(n_OL_rows)); CntrPar%OL_GenTq = 0.0
            ALLOCATE(CntrPar%OL_YawRate(n_OL_rows)); CntrPar%OL_YawRate = 0.0
            ALLOCATE(CntrPar%OL_Azimuth(n_OL_rows)); CntrPar%OL_Azimuth = 0.0
            ALLOCATE(CntrPar%OL_R_Speed(n_OL_rows)); CntrPar%OL_R_Speed = 0.0
            ALLOCATE(CntrPar%OL_R_Torque(n_OL_rows)); CntrPar%OL_R_Torque = 0.0
            ALLOCATE(CntrPar%OL_R_Pitch(n_OL_rows)); CntrPar%OL_R_Pitch = 0.0
            ! OL_CableControl and OL_StructControl allocated inside C++ pass 2
            ! (sizes depend on counting non-zero Ind_CableControl/Ind_StructControl)
        END IF

        ! ============================================================
        ! PASS 2: Fill arrays + computed constants + OL loading
        ! ============================================================
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL readcontrolparameterfilesub_pass2_c( &
            C_LOC(CntrPar_view), C_LOC(LocalVar), &
            filename_c, priPath_c, C_LOC(ErrVar))

        ! Copy scalars again (computed constants modified in pass 2)
        CALL vit_copy_scalars_to_cntrpar(CntrPar_view, CntrPar)
