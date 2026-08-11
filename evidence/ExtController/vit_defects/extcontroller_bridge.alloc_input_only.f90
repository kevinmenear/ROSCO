! VIT: Test-validate bridge for ExtController
! Allows C++ test harness to call the original Fortran function.
! Handles C↔Fortran type conversions for derived types and CHARACTER.
    SUBROUTINE extcontroller_f90(avrSWAP, vit_CntrPar_ZMQ_ID, vit_CntrPar_LoggingLevel, &
        vit_CntrPar_Echo, vit_CntrPar_Ext_Interface, vit_CntrPar_DT_Out, vit_CntrPar_n_DT_Out, &
        vit_CntrPar_n_DT_ZMQ, vit_CntrPar_F_LPFType, vit_CntrPar_F_LPFCornerFreq, &
        vit_CntrPar_F_LPFDamping, vit_CntrPar_F_NumNotchFilts, vit_CntrPar_F_GenSpdNotch_N, &
        vit_CntrPar_F_GenSpdNotch_Ind_ptr, vit_CntrPar_F_GenSpdNotch_Ind_n, &
        vit_CntrPar_F_TwrTopNotch_N, vit_CntrPar_F_TwrTopNotch_Ind_ptr, &
        vit_CntrPar_F_TwrTopNotch_Ind_n, vit_CntrPar_F_NotchFreqs_ptr, vit_CntrPar_F_NotchFreqs_n, &
        vit_CntrPar_F_NotchBetaNum_ptr, vit_CntrPar_F_NotchBetaNum_n, &
        vit_CntrPar_F_NotchBetaDen_ptr, vit_CntrPar_F_NotchBetaDen_n, vit_CntrPar_F_SSCornerFreq, &
        vit_CntrPar_F_WECornerFreq, vit_CntrPar_F_FlCornerFreq_ptr, vit_CntrPar_F_FlCornerFreq_n, &
        vit_CntrPar_F_FlHighPassFreq, vit_CntrPar_F_YawErr, vit_CntrPar_F_FlpCornerFreq_ptr, &
        vit_CntrPar_F_FlpCornerFreq_n, vit_CntrPar_F_VSRefSpdCornerFreq, vit_CntrPar_TRA_Mode, &
        vit_CntrPar_TRA_ExclSpeed, vit_CntrPar_TRA_ExclBand, vit_CntrPar_TRA_RateLimit, &
        vit_CntrPar_TD_Mode, vit_CntrPar_FA_HPFCornerFreq, vit_CntrPar_FA_IntSat, &
        vit_CntrPar_FA_KI, vit_CntrPar_IPC_ControlMode, vit_CntrPar_IPC_Vramp_ptr, &
        vit_CntrPar_IPC_Vramp_n, vit_CntrPar_IPC_IntSat, vit_CntrPar_IPC_SatMode, &
        vit_CntrPar_IPC_KP_ptr, vit_CntrPar_IPC_KP_n, vit_CntrPar_IPC_KI_ptr, vit_CntrPar_IPC_KI_n, &
        vit_CntrPar_IPC_aziOffset_ptr, vit_CntrPar_IPC_aziOffset_n, vit_CntrPar_IPC_CornerFreqAct, &
        vit_CntrPar_PC_ControlMode, vit_CntrPar_PC_GS_n, vit_CntrPar_PC_GS_angles_ptr, &
        vit_CntrPar_PC_GS_angles_n, vit_CntrPar_PC_GS_KP_ptr, vit_CntrPar_PC_GS_KP_n, &
        vit_CntrPar_PC_GS_KI_ptr, vit_CntrPar_PC_GS_KI_n, vit_CntrPar_PC_GS_KD_ptr, &
        vit_CntrPar_PC_GS_KD_n, vit_CntrPar_PC_GS_TF_ptr, vit_CntrPar_PC_GS_TF_n, &
        vit_CntrPar_PC_MaxPit, vit_CntrPar_PC_MinPit, vit_CntrPar_PC_MaxRat, vit_CntrPar_PC_MinRat, &
        vit_CntrPar_PC_RefSpd, vit_CntrPar_PC_FinePit, vit_CntrPar_PC_Switch, &
        vit_CntrPar_VS_ControlMode, vit_CntrPar_VS_ConstPower, vit_CntrPar_VS_FBP, &
        vit_CntrPar_VS_GenEff, vit_CntrPar_VS_ArSatTq, vit_CntrPar_VS_MaxRat, vit_CntrPar_VS_MaxTq, &
        vit_CntrPar_VS_MinTq, vit_CntrPar_VS_MinOMSpd, vit_CntrPar_VS_Rgn2K, vit_CntrPar_VS_RtPwr, &
        vit_CntrPar_VS_RtTq, vit_CntrPar_VS_RefSpd, vit_CntrPar_VS_n, vit_CntrPar_VS_KP_ptr, &
        vit_CntrPar_VS_KP_n, vit_CntrPar_VS_KI_ptr, vit_CntrPar_VS_KI_n, vit_CntrPar_VS_TSRopt, &
        vit_CntrPar_VS_FBP_n, vit_CntrPar_VS_FBP_U_ptr, vit_CntrPar_VS_FBP_U_n, &
        vit_CntrPar_VS_FBP_Omega_ptr, vit_CntrPar_VS_FBP_Omega_n, vit_CntrPar_VS_FBP_Tau_ptr, &
        vit_CntrPar_VS_FBP_Tau_n, vit_CntrPar_SS_Mode, vit_CntrPar_SS_VSGain, &
        vit_CntrPar_SS_PCGain, vit_CntrPar_PRC_Mode, vit_CntrPar_PRC_Comm, &
        vit_CntrPar_PRC_WindSpeeds_ptr, vit_CntrPar_PRC_WindSpeeds_n, &
        vit_CntrPar_PRC_GenSpeeds_ptr, vit_CntrPar_PRC_GenSpeeds_n, vit_CntrPar_PRC_n, &
        vit_CntrPar_PRC_LPF_Freq, vit_CntrPar_PRC_R_Torque, vit_CntrPar_PRC_R_Speed, &
        vit_CntrPar_PRC_R_Pitch, vit_CntrPar_PRC_Table_n, vit_CntrPar_PRC_Pitch_Table_ptr, &
        vit_CntrPar_PRC_Pitch_Table_n, vit_CntrPar_PRC_R_Table_ptr, vit_CntrPar_PRC_R_Table_n, &
        vit_CntrPar_WE_Mode, vit_CntrPar_WE_BladeRadius, vit_CntrPar_WE_CP_n, &
        vit_CntrPar_WE_CP_ptr, vit_CntrPar_WE_CP_n_v2, vit_CntrPar_WE_Gamma, &
        vit_CntrPar_WE_GearboxRatio, vit_CntrPar_WE_Jtot, vit_CntrPar_WE_RhoAir, &
        vit_CntrPar_PerfFileName, vit_CntrPar_PerfTableSize_ptr, vit_CntrPar_PerfTableSize_n, &
        vit_CntrPar_WE_FOPoles_N, vit_CntrPar_WE_FOPoles_v_ptr, vit_CntrPar_WE_FOPoles_v_n, &
        vit_CntrPar_WE_FOPoles_ptr, vit_CntrPar_WE_FOPoles_n_v2, vit_CntrPar_Y_ControlMode, &
        vit_CntrPar_Y_uSwitch, vit_CntrPar_Y_ErrThresh_ptr, vit_CntrPar_Y_ErrThresh_n, &
        vit_CntrPar_Y_Rate, vit_CntrPar_Y_MErrSet, vit_CntrPar_Y_IPC_IntSat, vit_CntrPar_Y_IPC_KP, &
        vit_CntrPar_Y_IPC_KI, vit_CntrPar_PS_Mode, vit_CntrPar_PS_BldPitchMin_N, &
        vit_CntrPar_PS_WindSpeeds_ptr, vit_CntrPar_PS_WindSpeeds_n, vit_CntrPar_PS_BldPitchMin_ptr, &
        vit_CntrPar_PS_BldPitchMin_n_v2, vit_CntrPar_SU_Mode, vit_CntrPar_SU_StartTime, &
        vit_CntrPar_SU_FW_MinDuration, vit_CntrPar_SU_RotorSpeedThresh, &
        vit_CntrPar_SU_RotorSpeedCornerFreq, vit_CntrPar_SU_LoadStages_N, &
        vit_CntrPar_SU_LoadStages_ptr, vit_CntrPar_SU_LoadStages_n_v2, &
        vit_CntrPar_SU_LoadRampDuration_ptr, vit_CntrPar_SU_LoadRampDuration_n, &
        vit_CntrPar_SU_LoadHoldDuration_ptr, vit_CntrPar_SU_LoadHoldDuration_n, &
        vit_CntrPar_SD_Mode, vit_CntrPar_SD_TimeActivate, vit_CntrPar_SD_EnablePitch, &
        vit_CntrPar_SD_EnableYawError, vit_CntrPar_SD_EnableGenSpeed, vit_CntrPar_SD_EnableTime, &
        vit_CntrPar_SD_MaxPit, vit_CntrPar_SD_PitchCornerFreq, vit_CntrPar_SD_MaxYawError, &
        vit_CntrPar_SD_YawErrorCornerFreq, vit_CntrPar_SD_MaxGenSpd, &
        vit_CntrPar_SD_GenSpdCornerFreq, vit_CntrPar_SD_Time, vit_CntrPar_SD_Method, &
        vit_CntrPar_SD_MaxTorqueRate_ptr, vit_CntrPar_SD_MaxTorqueRate_n, &
        vit_CntrPar_SD_MaxPitchRate_ptr, vit_CntrPar_SD_MaxPitchRate_n, &
        vit_CntrPar_SD_StagePitch_ptr, vit_CntrPar_SD_StagePitch_n, vit_CntrPar_SD_StageTime_ptr, &
        vit_CntrPar_SD_StageTime_n, vit_CntrPar_SD_Stage_N, vit_CntrPar_Fl_Mode, vit_CntrPar_Fl_n, &
        vit_CntrPar_Fl_Kp_ptr, vit_CntrPar_Fl_Kp_n, vit_CntrPar_Fl_U_ptr, vit_CntrPar_Fl_U_n, &
        vit_CntrPar_Flp_Mode, vit_CntrPar_Flp_Angle, vit_CntrPar_Flp_Kp, vit_CntrPar_Flp_Ki, &
        vit_CntrPar_Flp_MaxPit, vit_CntrPar_OL_Filename, vit_CntrPar_OL_Mode, &
        vit_CntrPar_OL_BP_Mode, vit_CntrPar_OL_BP_FiltFreq, vit_CntrPar_Ind_Breakpoint, &
        vit_CntrPar_Ind_BldPitch_ptr, vit_CntrPar_Ind_BldPitch_n, vit_CntrPar_Ind_GenTq, &
        vit_CntrPar_Ind_YawRate, vit_CntrPar_Ind_R_Speed, vit_CntrPar_Ind_R_Torque, &
        vit_CntrPar_Ind_R_Pitch, vit_CntrPar_Ind_Azimuth, vit_CntrPar_RP_Gains_ptr, &
        vit_CntrPar_RP_Gains_n, vit_CntrPar_Ind_CableControl_ptr, vit_CntrPar_Ind_CableControl_n, &
        vit_CntrPar_Ind_StructControl_ptr, vit_CntrPar_Ind_StructControl_n, &
        vit_CntrPar_OL_Breakpoints_ptr, vit_CntrPar_OL_Breakpoints_n, vit_CntrPar_OL_BldPitch1_ptr, &
        vit_CntrPar_OL_BldPitch1_n, vit_CntrPar_OL_BldPitch2_ptr, vit_CntrPar_OL_BldPitch2_n, &
        vit_CntrPar_OL_BldPitch3_ptr, vit_CntrPar_OL_BldPitch3_n, vit_CntrPar_OL_CableControl_ptr, &
        vit_CntrPar_OL_CableControl_rows, vit_CntrPar_OL_CableControl_cols, &
        vit_CntrPar_OL_StructControl_ptr, vit_CntrPar_OL_StructControl_rows, &
        vit_CntrPar_OL_StructControl_cols, vit_CntrPar_OL_GenTq_ptr, vit_CntrPar_OL_GenTq_n, &
        vit_CntrPar_OL_YawRate_ptr, vit_CntrPar_OL_YawRate_n, vit_CntrPar_OL_Azimuth_ptr, &
        vit_CntrPar_OL_Azimuth_n, vit_CntrPar_OL_R_Speed_ptr, vit_CntrPar_OL_R_Speed_n, &
        vit_CntrPar_OL_R_Torque_ptr, vit_CntrPar_OL_R_Torque_n, vit_CntrPar_OL_R_Pitch_ptr, &
        vit_CntrPar_OL_R_Pitch_n, vit_CntrPar_OL_Channels_ptr, vit_CntrPar_OL_Channels_rows, &
        vit_CntrPar_OL_Channels_cols, vit_CntrPar_PA_Mode, vit_CntrPar_PA_CornerFreq, &
        vit_CntrPar_PA_Damping, vit_CntrPar_AWC_Mode, vit_CntrPar_AWC_NumModes, &
        vit_CntrPar_AWC_n_ptr, vit_CntrPar_AWC_n_n, vit_CntrPar_AWC_harmonic_ptr, &
        vit_CntrPar_AWC_harmonic_n, vit_CntrPar_AWC_freq_ptr, vit_CntrPar_AWC_freq_n, &
        vit_CntrPar_AWC_amp_ptr, vit_CntrPar_AWC_amp_n, vit_CntrPar_AWC_clockangle_ptr, &
        vit_CntrPar_AWC_clockangle_n, vit_CntrPar_AWC_phaseoffset, vit_CntrPar_AWC_CntrGains_ptr, &
        vit_CntrPar_AWC_CntrGains_n, vit_CntrPar_PF_Mode, vit_CntrPar_PF_Offsets_ptr, &
        vit_CntrPar_PF_Offsets_n, vit_CntrPar_PF_TimeStuck_ptr, vit_CntrPar_PF_TimeStuck_n, &
        vit_CntrPar_Ext_Mode, vit_CntrPar_DLL_FileName, vit_CntrPar_DLL_InFile, &
        vit_CntrPar_DLL_ProcName, vit_CntrPar_ZMQ_Mode, vit_CntrPar_ZMQ_CommAddress, &
        vit_CntrPar_ZMQ_UpdatePeriod, vit_CntrPar_CC_Mode, vit_CntrPar_CC_Group_N, &
        vit_CntrPar_CC_ActTau, vit_CntrPar_CC_GroupIndex_ptr, vit_CntrPar_CC_GroupIndex_n, &
        vit_CntrPar_StC_Mode, vit_CntrPar_StC_Group_N, vit_CntrPar_StC_GroupIndex_ptr, &
        vit_CntrPar_StC_GroupIndex_n, vit_CntrPar_PC_RtTq99, vit_CntrPar_VS_MaxOMTq, &
        vit_CntrPar_VS_MinOMTq, vit_LocalVar_iStatus, vit_LocalVar_AlreadyInitialized, &
        vit_LocalVar_RestartWSE, vit_LocalVar_Time, vit_LocalVar_DT, vit_LocalVar_WriteThisStep, &
        vit_LocalVar_n_DT, vit_LocalVar_Time_Last, vit_LocalVar_VS_GenPwr, vit_LocalVar_GenSpeed, &
        vit_LocalVar_RotSpeed, vit_LocalVar_NacHeading, vit_LocalVar_NacVane, &
        vit_LocalVar_NacVaneF, vit_LocalVar_WindDir, vit_LocalVar_HorWindV, &
        vit_LocalVar_HorWindV_F, vit_LocalVar_rootMOOP, vit_LocalVar_rootMOOPF, &
        vit_LocalVar_BlPitch, vit_LocalVar_BlPitchCMeas, vit_LocalVar_Azimuth, &
        vit_LocalVar_OL_Azimuth, vit_LocalVar_AzUnwrapped, vit_LocalVar_AzError, &
        vit_LocalVar_GenTqAz, vit_LocalVar_AzBuffer, vit_LocalVar_NumBl, vit_LocalVar_FA_Acc_TT, &
        vit_LocalVar_SS_Acc_TT, vit_LocalVar_FA_Acc_Nac, vit_LocalVar_NacIMU_FA_RAcc, &
        vit_LocalVar_FA_AccHPF, vit_LocalVar_FA_AccHPFI, vit_LocalVar_FA_PitCom, &
        vit_LocalVar_VS_RefSpd, vit_LocalVar_VS_RefSpd_TSR, vit_LocalVar_VS_RefSpd_TRA, &
        vit_LocalVar_VS_RefSpd_RL, vit_LocalVar_PC_RefSpd, vit_LocalVar_PC_RefSpd_SS, &
        vit_LocalVar_PC_RefSpd_PRC, vit_LocalVar_RotSpeedF, vit_LocalVar_GenSpeedF, &
        vit_LocalVar_GenTq, vit_LocalVar_GenTqMeas, vit_LocalVar_GenArTq, vit_LocalVar_GenBrTq, &
        vit_LocalVar_VS_KOmega2_GenTq, vit_LocalVar_VS_ConstPwr_GenTq, vit_LocalVar_IPC_PitComF, &
        vit_LocalVar_PC_KP, vit_LocalVar_PC_KI, vit_LocalVar_PC_KD, vit_LocalVar_PC_TF, &
        vit_LocalVar_PC_MaxPit, vit_LocalVar_PC_MinPit, vit_LocalVar_PC_PitComT, &
        vit_LocalVar_PC_PitComT_Last, vit_LocalVar_BlPitchCMeasF, vit_LocalVar_PC_PitComT_IPC, &
        vit_LocalVar_PC_PwrErr, vit_LocalVar_PC_SpdErr, vit_LocalVar_IPC_AxisTilt_1P, &
        vit_LocalVar_IPC_AxisYaw_1P, vit_LocalVar_IPC_AxisTilt_2P, vit_LocalVar_IPC_AxisYaw_2P, &
        vit_LocalVar_axisTilt_1P, vit_LocalVar_axisYaw_1P, vit_LocalVar_axisYawF_1P, &
        vit_LocalVar_axisTilt_2P, vit_LocalVar_axisYaw_2P, vit_LocalVar_axisYawF_2P, &
        vit_LocalVar_IPC_KI, vit_LocalVar_IPC_KP, vit_LocalVar_IPC_IntSat, vit_LocalVar_PC_State, &
        vit_LocalVar_PitCom, vit_LocalVar_PitCom_SD, vit_LocalVar_PitComAct, &
        vit_LocalVar_SS_DelOmegaF, vit_LocalVar_TestType, vit_LocalVar_Kp_Float, &
        vit_LocalVar_VS_MaxTq, vit_LocalVar_VS_LastGenTrq, vit_LocalVar_VS_LastGenPwr, &
        vit_LocalVar_VS_MechGenPwr, vit_LocalVar_VS_SpdErrAr, vit_LocalVar_VS_SpdErrBr, &
        vit_LocalVar_VS_SpdErr, vit_LocalVar_VS_State, vit_LocalVar_VS_Rgn3Pitch, &
        vit_LocalVar_WE_Vw, vit_LocalVar_WE_Vw_F, vit_LocalVar_WE_VwI, vit_LocalVar_WE_VwIdot, &
        vit_LocalVar_WE_Op, vit_LocalVar_WE_Op_Last, vit_LocalVar_VS_LastGenTrqF, &
        vit_LocalVar_PRC_WSE_F, vit_LocalVar_PRC_R_Speed, vit_LocalVar_PRC_R_Torque, &
        vit_LocalVar_PRC_R_Pitch, vit_LocalVar_PRC_R_Total, vit_LocalVar_PRC_Min_Pitch, &
        vit_LocalVar_PS_Min_Pitch, vit_LocalVar_OL_Index, vit_LocalVar_SU_Stage, &
        vit_LocalVar_SU_LoadStageStartTime, vit_LocalVar_SU_RotSpeedF, vit_LocalVar_SD_Trigger, &
        vit_LocalVar_SD_BlPitchF, vit_LocalVar_SD_NacVaneF, vit_LocalVar_SD_GenSpeedF, &
        vit_LocalVar_SD_Stage, vit_LocalVar_SD_StageStartTime, vit_LocalVar_SD_MaxPitchRate, &
        vit_LocalVar_SD_MaxTorqueRate, vit_LocalVar_GenTq_SD, vit_LocalVar_Fl_PitCom, &
        vit_LocalVar_NACIMU_FA_AccF, vit_LocalVar_FA_AccF, vit_LocalVar_FA_Hist, &
        vit_LocalVar_TRA_LastRefSpd, vit_LocalVar_VS_RefSpeed, vit_LocalVar_PtfmTDX, &
        vit_LocalVar_PtfmTDY, vit_LocalVar_PtfmTDZ, vit_LocalVar_PtfmRDX, vit_LocalVar_PtfmRDY, &
        vit_LocalVar_PtfmRDZ, vit_LocalVar_PtfmTVX, vit_LocalVar_PtfmTVY, vit_LocalVar_PtfmTVZ, &
        vit_LocalVar_PtfmRVX, vit_LocalVar_PtfmRVY, vit_LocalVar_PtfmRVZ, vit_LocalVar_PtfmTAX, &
        vit_LocalVar_PtfmTAY, vit_LocalVar_PtfmTAZ, vit_LocalVar_PtfmRAX, vit_LocalVar_PtfmRAY, &
        vit_LocalVar_PtfmRAZ, vit_LocalVar_CC_DesiredL, vit_LocalVar_CC_ActuatedL, &
        vit_LocalVar_CC_ActuatedDL, vit_LocalVar_StC_Input, vit_LocalVar_Flp_Angle, &
        vit_LocalVar_RootMyb_Last, vit_LocalVar_ACC_INFILE_SIZE, vit_LocalVar_ACC_INFILE_ptr, &
        vit_LocalVar_ACC_INFILE_n, vit_LocalVar_restart, vit_LocalVar_AWC_complexangle, &
        vit_LocalVar_TiltMean, vit_LocalVar_YawMean, vit_LocalVar_ZMQ_ID, &
        vit_LocalVar_ZMQ_YawOffset, vit_LocalVar_ZMQ_TorqueOffset, vit_LocalVar_ZMQ_PitOffset, &
        vit_LocalVar_ZMQ_R_Speed, vit_LocalVar_ZMQ_R_Torque, vit_LocalVar_ZMQ_R_Pitch, &
        vit_LocalVar_WE_ptr, vit_LocalVar_FP_ptr, vit_LocalVar_piP_ptr, vit_LocalVar_resP_ptr, &
        vit_LocalVar_rlP_ptr, vit_ExtDLL_avrSWAP_ptr, vit_ExtDLL_avrSWAP_n, vit_ErrVar_size_avcMSG, &
        vit_ErrVar_aviFAIL, vit_ErrVar_ErrStat, vit_ErrVar_ErrMsg, vit_ErrVar_ErrMsg_n, &
        vit_ErrVar_ErrMsg_cap) &
        BIND(C, NAME='extcontroller_f90')
        USE ISO_C_BINDING
        USE, INTRINSIC :: ISO_FORTRAN_ENV, ONLY: ERROR_UNIT
        USE ExtControl
        USE Functions
        USE ROSCO_Types
        USE SysSubs
        USE Constants
        IMPLICIT NONE
        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
        INTEGER(C_INT), VALUE :: vit_CntrPar_ZMQ_ID
        INTEGER(C_INT), VALUE :: vit_CntrPar_LoggingLevel
        INTEGER(C_INT), VALUE :: vit_CntrPar_Echo
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ext_Interface
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_DT_Out
        INTEGER(C_INT), VALUE :: vit_CntrPar_n_DT_Out
        INTEGER(C_INT), VALUE :: vit_CntrPar_n_DT_ZMQ
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_LPFType
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_LPFCornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_LPFDamping
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NumNotchFilts
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_GenSpdNotch_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_GenSpdNotch_Ind_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_GenSpdNotch_Ind_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_TwrTopNotch_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_TwrTopNotch_Ind_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_TwrTopNotch_Ind_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_NotchFreqs_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NotchFreqs_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_NotchBetaNum_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NotchBetaNum_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_NotchBetaDen_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NotchBetaDen_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_SSCornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_WECornerFreq
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_FlCornerFreq_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_FlCornerFreq_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_FlHighPassFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_YawErr
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_FlpCornerFreq_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_FlpCornerFreq_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_F_VSRefSpdCornerFreq
        INTEGER(C_INT), VALUE :: vit_CntrPar_TRA_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_TRA_ExclSpeed
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_TRA_ExclBand
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_TRA_RateLimit
        INTEGER(C_INT), VALUE :: vit_CntrPar_TD_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_FA_HPFCornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_FA_IntSat
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_FA_KI
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_ControlMode
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_Vramp_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_Vramp_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_IPC_IntSat
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_SatMode
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_KP_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_KP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_KI_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_KI_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_aziOffset_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_aziOffset_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_IPC_CornerFreqAct
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_ControlMode
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_angles_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_angles_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_KP_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_KP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_KI_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_KI_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_KD_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_KD_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_TF_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_TF_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_MaxPit
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_MinPit
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_MaxRat
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_MinRat
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_RefSpd
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_FinePit
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_Switch
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_ControlMode
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_ConstPower
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_GenEff
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_ArSatTq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_MaxRat
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_MaxTq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_MinTq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_MinOMSpd
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_Rgn2K
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_RtPwr
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_RtTq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_RefSpd
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_KP_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_KP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_KI_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_KI_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_TSRopt
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_FBP_U_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_U_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_FBP_Omega_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_Omega_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_FBP_Tau_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_Tau_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SS_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SS_VSGain
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SS_PCGain
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_Comm
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_WindSpeeds_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_WindSpeeds_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_GenSpeeds_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_GenSpeeds_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PRC_LPF_Freq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PRC_R_Torque
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PRC_R_Speed
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PRC_R_Pitch
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_Table_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_Pitch_Table_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_Pitch_Table_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_R_Table_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_R_Table_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_WE_BladeRadius
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_CP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_WE_CP_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_CP_n_v2
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_WE_Gamma
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_WE_GearboxRatio
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_WE_Jtot
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_WE_RhoAir
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_PerfFileName(1024)
        TYPE(C_PTR), VALUE :: vit_CntrPar_PerfTableSize_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PerfTableSize_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_FOPoles_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_WE_FOPoles_v_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_FOPoles_v_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_WE_FOPoles_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_FOPoles_n_v2
        INTEGER(C_INT), VALUE :: vit_CntrPar_Y_ControlMode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Y_uSwitch
        TYPE(C_PTR), VALUE :: vit_CntrPar_Y_ErrThresh_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_Y_ErrThresh_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Y_Rate
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Y_MErrSet
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Y_IPC_IntSat
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Y_IPC_KP
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Y_IPC_KI
        INTEGER(C_INT), VALUE :: vit_CntrPar_PS_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_PS_BldPitchMin_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_PS_WindSpeeds_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PS_WindSpeeds_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PS_BldPitchMin_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PS_BldPitchMin_n_v2
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SU_StartTime
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SU_FW_MinDuration
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SU_RotorSpeedThresh
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SU_RotorSpeedCornerFreq
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadStages_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_SU_LoadStages_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadStages_n_v2
        TYPE(C_PTR), VALUE :: vit_CntrPar_SU_LoadRampDuration_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadRampDuration_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_SU_LoadHoldDuration_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadHoldDuration_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_TimeActivate
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_EnablePitch
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_EnableYawError
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_EnableGenSpeed
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_EnableTime
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_MaxPit
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_PitchCornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_MaxYawError
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_YawErrorCornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_MaxGenSpd
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_GenSpdCornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_SD_Time
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_Method
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_MaxTorqueRate_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_MaxTorqueRate_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_MaxPitchRate_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_MaxPitchRate_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_StagePitch_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_StagePitch_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_StageTime_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_StageTime_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_Stage_N
        INTEGER(C_INT), VALUE :: vit_CntrPar_Fl_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_Fl_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_Fl_Kp_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_Fl_Kp_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_Fl_U_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_Fl_U_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Flp_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Flp_Angle
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Flp_Kp
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Flp_Ki
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_Flp_MaxPit
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_OL_Filename(1024)
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BP_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_OL_BP_FiltFreq
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_Breakpoint
        TYPE(C_PTR), VALUE :: vit_CntrPar_Ind_BldPitch_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_BldPitch_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_GenTq
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_YawRate
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_R_Speed
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_R_Torque
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_R_Pitch
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_Azimuth
        TYPE(C_PTR), VALUE :: vit_CntrPar_RP_Gains_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_RP_Gains_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_Ind_CableControl_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_CableControl_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_Ind_StructControl_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_StructControl_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_Breakpoints_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Breakpoints_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_BldPitch1_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BldPitch1_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_BldPitch2_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BldPitch2_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_BldPitch3_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BldPitch3_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_CableControl_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_CableControl_rows
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_CableControl_cols
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_StructControl_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_StructControl_rows
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_StructControl_cols
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_GenTq_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_GenTq_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_YawRate_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_YawRate_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_Azimuth_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Azimuth_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_R_Speed_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_R_Speed_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_R_Torque_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_R_Torque_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_R_Pitch_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_R_Pitch_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_Channels_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Channels_rows
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Channels_cols
        INTEGER(C_INT), VALUE :: vit_CntrPar_PA_Mode
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PA_CornerFreq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PA_Damping
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_NumModes
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_n_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_n_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_harmonic_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_harmonic_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_freq_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_freq_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_amp_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_amp_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_clockangle_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_clockangle_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_AWC_phaseoffset
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_CntrGains_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_CntrGains_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PF_Mode
        TYPE(C_PTR), VALUE :: vit_CntrPar_PF_Offsets_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PF_Offsets_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PF_TimeStuck_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_PF_TimeStuck_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ext_Mode
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_DLL_FileName(1024)
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_DLL_InFile(1024)
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_DLL_ProcName(1024)
        INTEGER(C_INT), VALUE :: vit_CntrPar_ZMQ_Mode
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_ZMQ_CommAddress(256)
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_ZMQ_UpdatePeriod
        INTEGER(C_INT), VALUE :: vit_CntrPar_CC_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_CC_Group_N
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_CC_ActTau
        TYPE(C_PTR), VALUE :: vit_CntrPar_CC_GroupIndex_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_CC_GroupIndex_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_StC_Mode
        INTEGER(C_INT), VALUE :: vit_CntrPar_StC_Group_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_StC_GroupIndex_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_StC_GroupIndex_n
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_PC_RtTq99
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_MaxOMTq
        REAL(C_DOUBLE), VALUE :: vit_CntrPar_VS_MinOMTq
        INTEGER(C_INT), VALUE :: vit_LocalVar_iStatus
        INTEGER(C_INT), VALUE :: vit_LocalVar_AlreadyInitialized
        INTEGER(C_INT), VALUE :: vit_LocalVar_RestartWSE
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_Time
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_DT
        INTEGER(C_INT), VALUE :: vit_LocalVar_WriteThisStep
        INTEGER(C_INT), VALUE :: vit_LocalVar_n_DT
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_Time_Last
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_GenPwr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenSpeed
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_RotSpeed
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_NacHeading
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_NacVane
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_NacVaneF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_WindDir
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_HorWindV
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_HorWindV_F
        REAL(C_DOUBLE) :: vit_LocalVar_rootMOOP(3)
        REAL(C_DOUBLE) :: vit_LocalVar_rootMOOPF(3)
        REAL(C_DOUBLE) :: vit_LocalVar_BlPitch(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_BlPitchCMeas
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_Azimuth
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_OL_Azimuth
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_AzUnwrapped
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_AzError
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenTqAz
        REAL(C_DOUBLE) :: vit_LocalVar_AzBuffer(2)
        INTEGER(C_INT), VALUE :: vit_LocalVar_NumBl
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_FA_Acc_TT
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SS_Acc_TT
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_FA_Acc_Nac
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_NacIMU_FA_RAcc
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_FA_AccHPF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_FA_AccHPFI
        REAL(C_DOUBLE) :: vit_LocalVar_FA_PitCom(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_RefSpd
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_RefSpd_TSR
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_RefSpd_TRA
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_RefSpd_RL
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_RefSpd
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_RefSpd_SS
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_RefSpd_PRC
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_RotSpeedF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenSpeedF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenTq
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenTqMeas
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenArTq
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenBrTq
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_KOmega2_GenTq
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_ConstPwr_GenTq
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_PitComF(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_KP
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_KI
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_KD
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_TF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_MaxPit
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_MinPit
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_PitComT
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_PitComT_Last
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_BlPitchCMeasF
        REAL(C_DOUBLE) :: vit_LocalVar_PC_PitComT_IPC(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_PwrErr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PC_SpdErr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_IPC_AxisTilt_1P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_IPC_AxisYaw_1P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_IPC_AxisTilt_2P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_IPC_AxisYaw_2P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_axisTilt_1P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_axisYaw_1P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_axisYawF_1P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_axisTilt_2P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_axisYaw_2P
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_axisYawF_2P
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_KI(2)
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_KP(2)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_IPC_IntSat
        INTEGER(C_INT), VALUE :: vit_LocalVar_PC_State
        REAL(C_DOUBLE) :: vit_LocalVar_PitCom(3)
        REAL(C_DOUBLE) :: vit_LocalVar_PitCom_SD(3)
        REAL(C_DOUBLE) :: vit_LocalVar_PitComAct(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SS_DelOmegaF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_TestType
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_Kp_Float
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_MaxTq
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_LastGenTrq
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_LastGenPwr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_MechGenPwr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_SpdErrAr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_SpdErrBr
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_SpdErr
        INTEGER(C_INT), VALUE :: vit_LocalVar_VS_State
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_Rgn3Pitch
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_WE_Vw
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_WE_Vw_F
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_WE_VwI
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_WE_VwIdot
        INTEGER(C_INT), VALUE :: vit_LocalVar_WE_Op
        INTEGER(C_INT), VALUE :: vit_LocalVar_WE_Op_Last
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_LastGenTrqF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PRC_WSE_F
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PRC_R_Speed
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PRC_R_Torque
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PRC_R_Pitch
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PRC_R_Total
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PRC_Min_Pitch
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PS_Min_Pitch
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_OL_Index
        INTEGER(C_INT), VALUE :: vit_LocalVar_SU_Stage
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SU_LoadStageStartTime
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SU_RotSpeedF
        INTEGER(C_INT), VALUE :: vit_LocalVar_SD_Trigger
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SD_BlPitchF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SD_NacVaneF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SD_GenSpeedF
        INTEGER(C_INT), VALUE :: vit_LocalVar_SD_Stage
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SD_StageStartTime
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SD_MaxPitchRate
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_SD_MaxTorqueRate
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_GenTq_SD
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_Fl_PitCom
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_NACIMU_FA_AccF
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_FA_AccF
        INTEGER(C_INT), VALUE :: vit_LocalVar_FA_Hist
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_TRA_LastRefSpd
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_VS_RefSpeed
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTDX
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTDY
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTDZ
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRDX
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRDY
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRDZ
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTVX
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTVY
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTVZ
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRVX
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRVY
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRVZ
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTAX
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTAY
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmTAZ
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRAX
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRAY
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_PtfmRAZ
        REAL(C_DOUBLE) :: vit_LocalVar_CC_DesiredL(12)
        REAL(C_DOUBLE) :: vit_LocalVar_CC_ActuatedL(12)
        REAL(C_DOUBLE) :: vit_LocalVar_CC_ActuatedDL(12)
        REAL(C_DOUBLE) :: vit_LocalVar_StC_Input(12)
        REAL(C_DOUBLE) :: vit_LocalVar_Flp_Angle(3)
        REAL(C_DOUBLE) :: vit_LocalVar_RootMyb_Last(3)
        INTEGER(C_INT), VALUE :: vit_LocalVar_ACC_INFILE_SIZE
        TYPE(C_PTR), VALUE :: vit_LocalVar_ACC_INFILE_ptr
        INTEGER(C_INT), VALUE :: vit_LocalVar_ACC_INFILE_n
        INTEGER(C_INT), VALUE :: vit_LocalVar_restart
        COMPLEX(C_DOUBLE_COMPLEX) :: vit_LocalVar_AWC_complexangle(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_TiltMean
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_YawMean
        INTEGER(C_INT), VALUE :: vit_LocalVar_ZMQ_ID
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_ZMQ_YawOffset
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_ZMQ_TorqueOffset
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_PitOffset(3)
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_ZMQ_R_Speed
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_ZMQ_R_Torque
        REAL(C_DOUBLE), VALUE :: vit_LocalVar_ZMQ_R_Pitch
        TYPE(C_PTR), VALUE :: vit_LocalVar_WE_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_FP_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_piP_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_resP_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_rlP_ptr
        TYPE(C_PTR), VALUE :: vit_ExtDLL_avrSWAP_ptr
        INTEGER(C_INT), VALUE :: vit_ExtDLL_avrSWAP_n
        INTEGER(C_INT), VALUE :: vit_ErrVar_size_avcMSG
        INTEGER(C_INT), VALUE :: vit_ErrVar_aviFAIL
        INTEGER(C_INT), VALUE :: vit_ErrVar_ErrStat
        CHARACTER(KIND=C_CHAR) :: vit_ErrVar_ErrMsg(*)
        INTEGER(C_INT) :: vit_ErrVar_ErrMsg_n
        INTEGER(C_INT), VALUE :: vit_ErrVar_ErrMsg_cap
        INTEGER :: vit_ci_CntrPar_PerfFileName
        INTEGER :: vit_ci_CntrPar_OL_Filename
        INTEGER :: vit_ci_CntrPar_DLL_FileName
        INTEGER :: vit_ci_CntrPar_DLL_InFile
        INTEGER :: vit_ci_CntrPar_DLL_ProcName
        INTEGER :: vit_ci_CntrPar_ZMQ_CommAddress
        TYPE(ControlParameters) :: vit_local_CntrPar
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_F_GenSpdNotch_Ind(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_F_TwrTopNotch_Ind(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_F_NotchFreqs(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_F_NotchBetaNum(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_F_NotchBetaDen(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_F_FlCornerFreq(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_F_FlpCornerFreq(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_IPC_Vramp(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_IPC_KP(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_IPC_KI(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_IPC_aziOffset(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PC_GS_angles(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PC_GS_KP(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PC_GS_KI(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PC_GS_KD(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PC_GS_TF(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_VS_KP(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_VS_KI(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_VS_FBP_U(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_VS_FBP_Omega(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_VS_FBP_Tau(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PRC_WindSpeeds(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PRC_GenSpeeds(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PRC_Pitch_Table(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PRC_R_Table(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_WE_CP(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_PerfTableSize(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_WE_FOPoles_v(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_WE_FOPoles(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_Y_ErrThresh(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PS_WindSpeeds(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PS_BldPitchMin(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SU_LoadStages(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SU_LoadRampDuration(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SU_LoadHoldDuration(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SD_MaxTorqueRate(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SD_MaxPitchRate(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SD_StagePitch(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_SD_StageTime(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_Fl_Kp(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_Fl_U(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_Ind_BldPitch(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_RP_Gains(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_Ind_CableControl(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_Ind_StructControl(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_Breakpoints(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_BldPitch1(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_BldPitch2(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_BldPitch3(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_CableControl(:)
        INTEGER :: vit_i_OL_CableControl, vit_j_OL_CableControl
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_StructControl(:)
        INTEGER :: vit_i_OL_StructControl, vit_j_OL_StructControl
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_GenTq(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_YawRate(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_Azimuth(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_R_Speed(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_R_Torque(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_R_Pitch(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_OL_Channels(:)
        INTEGER :: vit_i_OL_Channels, vit_j_OL_Channels
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_AWC_n(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_AWC_harmonic(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_AWC_freq(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_AWC_amp(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_AWC_clockangle(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_AWC_CntrGains(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PF_Offsets(:)
        REAL(C_DOUBLE), POINTER :: vit_tmp_CntrPar_PF_TimeStuck(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_CC_GroupIndex(:)
        INTEGER(C_INT), POINTER :: vit_tmp_CntrPar_StC_GroupIndex(:)
        TYPE(LocalVariables) :: vit_local_LocalVar
        CHARACTER(C_CHAR), POINTER :: vit_tmp_LocalVar_ACC_INFILE(:)
        TYPE(WE), POINTER :: vit_nested_LocalVar_WE
        TYPE(FilterParameters), POINTER :: vit_nested_LocalVar_FP
        TYPE(piParams), POINTER :: vit_nested_LocalVar_piP
        TYPE(resParams), POINTER :: vit_nested_LocalVar_resP
        TYPE(rlParams), POINTER :: vit_nested_LocalVar_rlP
        TYPE(ExtControlType) :: vit_local_ExtDLL
        REAL(C_FLOAT), POINTER :: vit_tmp_ExtDLL_avrSWAP(:)
        INTEGER :: vit_dc_ErrVar_ErrMsg
        TYPE(ErrorVariables) :: vit_local_ErrVar
        vit_local_CntrPar%ZMQ_ID = vit_CntrPar_ZMQ_ID
        vit_local_CntrPar%LoggingLevel = vit_CntrPar_LoggingLevel
        vit_local_CntrPar%Echo = vit_CntrPar_Echo
        vit_local_CntrPar%Ext_Interface = vit_CntrPar_Ext_Interface
        vit_local_CntrPar%DT_Out = vit_CntrPar_DT_Out
        vit_local_CntrPar%n_DT_Out = vit_CntrPar_n_DT_Out
        vit_local_CntrPar%n_DT_ZMQ = vit_CntrPar_n_DT_ZMQ
        vit_local_CntrPar%F_LPFType = vit_CntrPar_F_LPFType
        vit_local_CntrPar%F_LPFCornerFreq = vit_CntrPar_F_LPFCornerFreq
        vit_local_CntrPar%F_LPFDamping = vit_CntrPar_F_LPFDamping
        vit_local_CntrPar%F_NumNotchFilts = vit_CntrPar_F_NumNotchFilts
        vit_local_CntrPar%F_GenSpdNotch_N = vit_CntrPar_F_GenSpdNotch_N
        IF (C_ASSOCIATED(vit_CntrPar_F_GenSpdNotch_Ind_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_GenSpdNotch_Ind(vit_CntrPar_F_GenSpdNotch_Ind_n))
            CALL C_F_POINTER(vit_CntrPar_F_GenSpdNotch_Ind_ptr, vit_tmp_CntrPar_F_GenSpdNotch_Ind, &
                [vit_CntrPar_F_GenSpdNotch_Ind_n])
            vit_local_CntrPar%F_GenSpdNotch_Ind(1:vit_CntrPar_F_GenSpdNotch_Ind_n) = &
                vit_tmp_CntrPar_F_GenSpdNotch_Ind(1:vit_CntrPar_F_GenSpdNotch_Ind_n)
        END IF
        vit_local_CntrPar%F_TwrTopNotch_N = vit_CntrPar_F_TwrTopNotch_N
        IF (C_ASSOCIATED(vit_CntrPar_F_TwrTopNotch_Ind_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_TwrTopNotch_Ind(vit_CntrPar_F_TwrTopNotch_Ind_n))
            CALL C_F_POINTER(vit_CntrPar_F_TwrTopNotch_Ind_ptr, vit_tmp_CntrPar_F_TwrTopNotch_Ind, &
                [vit_CntrPar_F_TwrTopNotch_Ind_n])
            vit_local_CntrPar%F_TwrTopNotch_Ind(1:vit_CntrPar_F_TwrTopNotch_Ind_n) = &
                vit_tmp_CntrPar_F_TwrTopNotch_Ind(1:vit_CntrPar_F_TwrTopNotch_Ind_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_F_NotchFreqs_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_NotchFreqs(vit_CntrPar_F_NotchFreqs_n))
            CALL C_F_POINTER(vit_CntrPar_F_NotchFreqs_ptr, vit_tmp_CntrPar_F_NotchFreqs, [vit_CntrPar_F_NotchFreqs_n])
            vit_local_CntrPar%F_NotchFreqs(1:vit_CntrPar_F_NotchFreqs_n) = &
                vit_tmp_CntrPar_F_NotchFreqs(1:vit_CntrPar_F_NotchFreqs_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_F_NotchBetaNum_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_NotchBetaNum(vit_CntrPar_F_NotchBetaNum_n))
            CALL C_F_POINTER(vit_CntrPar_F_NotchBetaNum_ptr, vit_tmp_CntrPar_F_NotchBetaNum, &
                [vit_CntrPar_F_NotchBetaNum_n])
            vit_local_CntrPar%F_NotchBetaNum(1:vit_CntrPar_F_NotchBetaNum_n) = &
                vit_tmp_CntrPar_F_NotchBetaNum(1:vit_CntrPar_F_NotchBetaNum_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_F_NotchBetaDen_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_NotchBetaDen(vit_CntrPar_F_NotchBetaDen_n))
            CALL C_F_POINTER(vit_CntrPar_F_NotchBetaDen_ptr, vit_tmp_CntrPar_F_NotchBetaDen, &
                [vit_CntrPar_F_NotchBetaDen_n])
            vit_local_CntrPar%F_NotchBetaDen(1:vit_CntrPar_F_NotchBetaDen_n) = &
                vit_tmp_CntrPar_F_NotchBetaDen(1:vit_CntrPar_F_NotchBetaDen_n)
        END IF
        vit_local_CntrPar%F_SSCornerFreq = vit_CntrPar_F_SSCornerFreq
        vit_local_CntrPar%F_WECornerFreq = vit_CntrPar_F_WECornerFreq
        IF (C_ASSOCIATED(vit_CntrPar_F_FlCornerFreq_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_FlCornerFreq(vit_CntrPar_F_FlCornerFreq_n))
            CALL C_F_POINTER(vit_CntrPar_F_FlCornerFreq_ptr, vit_tmp_CntrPar_F_FlCornerFreq, &
                [vit_CntrPar_F_FlCornerFreq_n])
            vit_local_CntrPar%F_FlCornerFreq(1:vit_CntrPar_F_FlCornerFreq_n) = &
                vit_tmp_CntrPar_F_FlCornerFreq(1:vit_CntrPar_F_FlCornerFreq_n)
        END IF
        vit_local_CntrPar%F_FlHighPassFreq = vit_CntrPar_F_FlHighPassFreq
        vit_local_CntrPar%F_YawErr = vit_CntrPar_F_YawErr
        IF (C_ASSOCIATED(vit_CntrPar_F_FlpCornerFreq_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%F_FlpCornerFreq(vit_CntrPar_F_FlpCornerFreq_n))
            CALL C_F_POINTER(vit_CntrPar_F_FlpCornerFreq_ptr, vit_tmp_CntrPar_F_FlpCornerFreq, &
                [vit_CntrPar_F_FlpCornerFreq_n])
            vit_local_CntrPar%F_FlpCornerFreq(1:vit_CntrPar_F_FlpCornerFreq_n) = &
                vit_tmp_CntrPar_F_FlpCornerFreq(1:vit_CntrPar_F_FlpCornerFreq_n)
        END IF
        vit_local_CntrPar%F_VSRefSpdCornerFreq = vit_CntrPar_F_VSRefSpdCornerFreq
        vit_local_CntrPar%TRA_Mode = vit_CntrPar_TRA_Mode
        vit_local_CntrPar%TRA_ExclSpeed = vit_CntrPar_TRA_ExclSpeed
        vit_local_CntrPar%TRA_ExclBand = vit_CntrPar_TRA_ExclBand
        vit_local_CntrPar%TRA_RateLimit = vit_CntrPar_TRA_RateLimit
        vit_local_CntrPar%TD_Mode = vit_CntrPar_TD_Mode
        vit_local_CntrPar%FA_HPFCornerFreq = vit_CntrPar_FA_HPFCornerFreq
        vit_local_CntrPar%FA_IntSat = vit_CntrPar_FA_IntSat
        vit_local_CntrPar%FA_KI = vit_CntrPar_FA_KI
        vit_local_CntrPar%IPC_ControlMode = vit_CntrPar_IPC_ControlMode
        IF (C_ASSOCIATED(vit_CntrPar_IPC_Vramp_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%IPC_Vramp(vit_CntrPar_IPC_Vramp_n))
            CALL C_F_POINTER(vit_CntrPar_IPC_Vramp_ptr, vit_tmp_CntrPar_IPC_Vramp, [vit_CntrPar_IPC_Vramp_n])
            vit_local_CntrPar%IPC_Vramp(1:vit_CntrPar_IPC_Vramp_n) = &
                vit_tmp_CntrPar_IPC_Vramp(1:vit_CntrPar_IPC_Vramp_n)
        END IF
        vit_local_CntrPar%IPC_IntSat = vit_CntrPar_IPC_IntSat
        vit_local_CntrPar%IPC_SatMode = vit_CntrPar_IPC_SatMode
        IF (C_ASSOCIATED(vit_CntrPar_IPC_KP_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%IPC_KP(vit_CntrPar_IPC_KP_n))
            CALL C_F_POINTER(vit_CntrPar_IPC_KP_ptr, vit_tmp_CntrPar_IPC_KP, [vit_CntrPar_IPC_KP_n])
            vit_local_CntrPar%IPC_KP(1:vit_CntrPar_IPC_KP_n) = vit_tmp_CntrPar_IPC_KP(1:vit_CntrPar_IPC_KP_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_IPC_KI_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%IPC_KI(vit_CntrPar_IPC_KI_n))
            CALL C_F_POINTER(vit_CntrPar_IPC_KI_ptr, vit_tmp_CntrPar_IPC_KI, [vit_CntrPar_IPC_KI_n])
            vit_local_CntrPar%IPC_KI(1:vit_CntrPar_IPC_KI_n) = vit_tmp_CntrPar_IPC_KI(1:vit_CntrPar_IPC_KI_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_IPC_aziOffset_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%IPC_aziOffset(vit_CntrPar_IPC_aziOffset_n))
            CALL C_F_POINTER(vit_CntrPar_IPC_aziOffset_ptr, vit_tmp_CntrPar_IPC_aziOffset, &
                [vit_CntrPar_IPC_aziOffset_n])
            vit_local_CntrPar%IPC_aziOffset(1:vit_CntrPar_IPC_aziOffset_n) = &
                vit_tmp_CntrPar_IPC_aziOffset(1:vit_CntrPar_IPC_aziOffset_n)
        END IF
        vit_local_CntrPar%IPC_CornerFreqAct = vit_CntrPar_IPC_CornerFreqAct
        vit_local_CntrPar%PC_ControlMode = vit_CntrPar_PC_ControlMode
        vit_local_CntrPar%PC_GS_n = vit_CntrPar_PC_GS_n
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_angles_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_angles(vit_CntrPar_PC_GS_angles_n))
            CALL C_F_POINTER(vit_CntrPar_PC_GS_angles_ptr, vit_tmp_CntrPar_PC_GS_angles, [vit_CntrPar_PC_GS_angles_n])
            vit_local_CntrPar%PC_GS_angles(1:vit_CntrPar_PC_GS_angles_n) = &
                vit_tmp_CntrPar_PC_GS_angles(1:vit_CntrPar_PC_GS_angles_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_KP_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_KP(vit_CntrPar_PC_GS_KP_n))
            CALL C_F_POINTER(vit_CntrPar_PC_GS_KP_ptr, vit_tmp_CntrPar_PC_GS_KP, [vit_CntrPar_PC_GS_KP_n])
            vit_local_CntrPar%PC_GS_KP(1:vit_CntrPar_PC_GS_KP_n) = vit_tmp_CntrPar_PC_GS_KP(1:vit_CntrPar_PC_GS_KP_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_KI_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_KI(vit_CntrPar_PC_GS_KI_n))
            CALL C_F_POINTER(vit_CntrPar_PC_GS_KI_ptr, vit_tmp_CntrPar_PC_GS_KI, [vit_CntrPar_PC_GS_KI_n])
            vit_local_CntrPar%PC_GS_KI(1:vit_CntrPar_PC_GS_KI_n) = vit_tmp_CntrPar_PC_GS_KI(1:vit_CntrPar_PC_GS_KI_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_KD_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_KD(vit_CntrPar_PC_GS_KD_n))
            CALL C_F_POINTER(vit_CntrPar_PC_GS_KD_ptr, vit_tmp_CntrPar_PC_GS_KD, [vit_CntrPar_PC_GS_KD_n])
            vit_local_CntrPar%PC_GS_KD(1:vit_CntrPar_PC_GS_KD_n) = vit_tmp_CntrPar_PC_GS_KD(1:vit_CntrPar_PC_GS_KD_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_TF_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_TF(vit_CntrPar_PC_GS_TF_n))
            CALL C_F_POINTER(vit_CntrPar_PC_GS_TF_ptr, vit_tmp_CntrPar_PC_GS_TF, [vit_CntrPar_PC_GS_TF_n])
            vit_local_CntrPar%PC_GS_TF(1:vit_CntrPar_PC_GS_TF_n) = vit_tmp_CntrPar_PC_GS_TF(1:vit_CntrPar_PC_GS_TF_n)
        END IF
        vit_local_CntrPar%PC_MaxPit = vit_CntrPar_PC_MaxPit
        vit_local_CntrPar%PC_MinPit = vit_CntrPar_PC_MinPit
        vit_local_CntrPar%PC_MaxRat = vit_CntrPar_PC_MaxRat
        vit_local_CntrPar%PC_MinRat = vit_CntrPar_PC_MinRat
        vit_local_CntrPar%PC_RefSpd = vit_CntrPar_PC_RefSpd
        vit_local_CntrPar%PC_FinePit = vit_CntrPar_PC_FinePit
        vit_local_CntrPar%PC_Switch = vit_CntrPar_PC_Switch
        vit_local_CntrPar%VS_ControlMode = vit_CntrPar_VS_ControlMode
        vit_local_CntrPar%VS_ConstPower = vit_CntrPar_VS_ConstPower
        vit_local_CntrPar%VS_FBP = vit_CntrPar_VS_FBP
        vit_local_CntrPar%VS_GenEff = vit_CntrPar_VS_GenEff
        vit_local_CntrPar%VS_ArSatTq = vit_CntrPar_VS_ArSatTq
        vit_local_CntrPar%VS_MaxRat = vit_CntrPar_VS_MaxRat
        vit_local_CntrPar%VS_MaxTq = vit_CntrPar_VS_MaxTq
        vit_local_CntrPar%VS_MinTq = vit_CntrPar_VS_MinTq
        vit_local_CntrPar%VS_MinOMSpd = vit_CntrPar_VS_MinOMSpd
        vit_local_CntrPar%VS_Rgn2K = vit_CntrPar_VS_Rgn2K
        vit_local_CntrPar%VS_RtPwr = vit_CntrPar_VS_RtPwr
        vit_local_CntrPar%VS_RtTq = vit_CntrPar_VS_RtTq
        vit_local_CntrPar%VS_RefSpd = vit_CntrPar_VS_RefSpd
        vit_local_CntrPar%VS_n = vit_CntrPar_VS_n
        IF (C_ASSOCIATED(vit_CntrPar_VS_KP_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%VS_KP(vit_CntrPar_VS_KP_n))
            CALL C_F_POINTER(vit_CntrPar_VS_KP_ptr, vit_tmp_CntrPar_VS_KP, [vit_CntrPar_VS_KP_n])
            vit_local_CntrPar%VS_KP(1:vit_CntrPar_VS_KP_n) = vit_tmp_CntrPar_VS_KP(1:vit_CntrPar_VS_KP_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_VS_KI_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%VS_KI(vit_CntrPar_VS_KI_n))
            CALL C_F_POINTER(vit_CntrPar_VS_KI_ptr, vit_tmp_CntrPar_VS_KI, [vit_CntrPar_VS_KI_n])
            vit_local_CntrPar%VS_KI(1:vit_CntrPar_VS_KI_n) = vit_tmp_CntrPar_VS_KI(1:vit_CntrPar_VS_KI_n)
        END IF
        vit_local_CntrPar%VS_TSRopt = vit_CntrPar_VS_TSRopt
        vit_local_CntrPar%VS_FBP_n = vit_CntrPar_VS_FBP_n
        IF (C_ASSOCIATED(vit_CntrPar_VS_FBP_U_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%VS_FBP_U(vit_CntrPar_VS_FBP_U_n))
            CALL C_F_POINTER(vit_CntrPar_VS_FBP_U_ptr, vit_tmp_CntrPar_VS_FBP_U, [vit_CntrPar_VS_FBP_U_n])
            vit_local_CntrPar%VS_FBP_U(1:vit_CntrPar_VS_FBP_U_n) = vit_tmp_CntrPar_VS_FBP_U(1:vit_CntrPar_VS_FBP_U_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_VS_FBP_Omega_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%VS_FBP_Omega(vit_CntrPar_VS_FBP_Omega_n))
            CALL C_F_POINTER(vit_CntrPar_VS_FBP_Omega_ptr, vit_tmp_CntrPar_VS_FBP_Omega, [vit_CntrPar_VS_FBP_Omega_n])
            vit_local_CntrPar%VS_FBP_Omega(1:vit_CntrPar_VS_FBP_Omega_n) = &
                vit_tmp_CntrPar_VS_FBP_Omega(1:vit_CntrPar_VS_FBP_Omega_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_VS_FBP_Tau_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%VS_FBP_Tau(vit_CntrPar_VS_FBP_Tau_n))
            CALL C_F_POINTER(vit_CntrPar_VS_FBP_Tau_ptr, vit_tmp_CntrPar_VS_FBP_Tau, [vit_CntrPar_VS_FBP_Tau_n])
            vit_local_CntrPar%VS_FBP_Tau(1:vit_CntrPar_VS_FBP_Tau_n) = &
                vit_tmp_CntrPar_VS_FBP_Tau(1:vit_CntrPar_VS_FBP_Tau_n)
        END IF
        vit_local_CntrPar%SS_Mode = vit_CntrPar_SS_Mode
        vit_local_CntrPar%SS_VSGain = vit_CntrPar_SS_VSGain
        vit_local_CntrPar%SS_PCGain = vit_CntrPar_SS_PCGain
        vit_local_CntrPar%PRC_Mode = vit_CntrPar_PRC_Mode
        vit_local_CntrPar%PRC_Comm = vit_CntrPar_PRC_Comm
        IF (C_ASSOCIATED(vit_CntrPar_PRC_WindSpeeds_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PRC_WindSpeeds(vit_CntrPar_PRC_WindSpeeds_n))
            CALL C_F_POINTER(vit_CntrPar_PRC_WindSpeeds_ptr, vit_tmp_CntrPar_PRC_WindSpeeds, &
                [vit_CntrPar_PRC_WindSpeeds_n])
            vit_local_CntrPar%PRC_WindSpeeds(1:vit_CntrPar_PRC_WindSpeeds_n) = &
                vit_tmp_CntrPar_PRC_WindSpeeds(1:vit_CntrPar_PRC_WindSpeeds_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PRC_GenSpeeds_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PRC_GenSpeeds(vit_CntrPar_PRC_GenSpeeds_n))
            CALL C_F_POINTER(vit_CntrPar_PRC_GenSpeeds_ptr, vit_tmp_CntrPar_PRC_GenSpeeds, &
                [vit_CntrPar_PRC_GenSpeeds_n])
            vit_local_CntrPar%PRC_GenSpeeds(1:vit_CntrPar_PRC_GenSpeeds_n) = &
                vit_tmp_CntrPar_PRC_GenSpeeds(1:vit_CntrPar_PRC_GenSpeeds_n)
        END IF
        vit_local_CntrPar%PRC_n = vit_CntrPar_PRC_n
        vit_local_CntrPar%PRC_LPF_Freq = vit_CntrPar_PRC_LPF_Freq
        vit_local_CntrPar%PRC_R_Torque = vit_CntrPar_PRC_R_Torque
        vit_local_CntrPar%PRC_R_Speed = vit_CntrPar_PRC_R_Speed
        vit_local_CntrPar%PRC_R_Pitch = vit_CntrPar_PRC_R_Pitch
        vit_local_CntrPar%PRC_Table_n = vit_CntrPar_PRC_Table_n
        IF (C_ASSOCIATED(vit_CntrPar_PRC_Pitch_Table_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PRC_Pitch_Table(vit_CntrPar_PRC_Pitch_Table_n))
            CALL C_F_POINTER(vit_CntrPar_PRC_Pitch_Table_ptr, vit_tmp_CntrPar_PRC_Pitch_Table, &
                [vit_CntrPar_PRC_Pitch_Table_n])
            vit_local_CntrPar%PRC_Pitch_Table(1:vit_CntrPar_PRC_Pitch_Table_n) = &
                vit_tmp_CntrPar_PRC_Pitch_Table(1:vit_CntrPar_PRC_Pitch_Table_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PRC_R_Table_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PRC_R_Table(vit_CntrPar_PRC_R_Table_n))
            CALL C_F_POINTER(vit_CntrPar_PRC_R_Table_ptr, vit_tmp_CntrPar_PRC_R_Table, [vit_CntrPar_PRC_R_Table_n])
            vit_local_CntrPar%PRC_R_Table(1:vit_CntrPar_PRC_R_Table_n) = &
                vit_tmp_CntrPar_PRC_R_Table(1:vit_CntrPar_PRC_R_Table_n)
        END IF
        vit_local_CntrPar%WE_Mode = vit_CntrPar_WE_Mode
        vit_local_CntrPar%WE_BladeRadius = vit_CntrPar_WE_BladeRadius
        vit_local_CntrPar%WE_CP_n = vit_CntrPar_WE_CP_n
        IF (C_ASSOCIATED(vit_CntrPar_WE_CP_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%WE_CP(vit_CntrPar_WE_CP_n_v2))
            CALL C_F_POINTER(vit_CntrPar_WE_CP_ptr, vit_tmp_CntrPar_WE_CP, [vit_CntrPar_WE_CP_n_v2])
            vit_local_CntrPar%WE_CP(1:vit_CntrPar_WE_CP_n_v2) = vit_tmp_CntrPar_WE_CP(1:vit_CntrPar_WE_CP_n_v2)
        END IF
        vit_local_CntrPar%WE_Gamma = vit_CntrPar_WE_Gamma
        vit_local_CntrPar%WE_GearboxRatio = vit_CntrPar_WE_GearboxRatio
        vit_local_CntrPar%WE_Jtot = vit_CntrPar_WE_Jtot
        vit_local_CntrPar%WE_RhoAir = vit_CntrPar_WE_RhoAir
        DO vit_ci_CntrPar_PerfFileName = 1, 1024
            vit_local_CntrPar%PerfFileName(vit_ci_CntrPar_PerfFileName:vit_ci_CntrPar_PerfFileName) = &
                vit_CntrPar_PerfFileName(vit_ci_CntrPar_PerfFileName)
        END DO
        IF (C_ASSOCIATED(vit_CntrPar_PerfTableSize_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PerfTableSize(vit_CntrPar_PerfTableSize_n))
            CALL C_F_POINTER(vit_CntrPar_PerfTableSize_ptr, vit_tmp_CntrPar_PerfTableSize, &
                [vit_CntrPar_PerfTableSize_n])
            vit_local_CntrPar%PerfTableSize(1:vit_CntrPar_PerfTableSize_n) = &
                vit_tmp_CntrPar_PerfTableSize(1:vit_CntrPar_PerfTableSize_n)
        END IF
        vit_local_CntrPar%WE_FOPoles_N = vit_CntrPar_WE_FOPoles_N
        IF (C_ASSOCIATED(vit_CntrPar_WE_FOPoles_v_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%WE_FOPoles_v(vit_CntrPar_WE_FOPoles_v_n))
            CALL C_F_POINTER(vit_CntrPar_WE_FOPoles_v_ptr, vit_tmp_CntrPar_WE_FOPoles_v, [vit_CntrPar_WE_FOPoles_v_n])
            vit_local_CntrPar%WE_FOPoles_v(1:vit_CntrPar_WE_FOPoles_v_n) = &
                vit_tmp_CntrPar_WE_FOPoles_v(1:vit_CntrPar_WE_FOPoles_v_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_WE_FOPoles_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%WE_FOPoles(vit_CntrPar_WE_FOPoles_n_v2))
            CALL C_F_POINTER(vit_CntrPar_WE_FOPoles_ptr, vit_tmp_CntrPar_WE_FOPoles, [vit_CntrPar_WE_FOPoles_n_v2])
            vit_local_CntrPar%WE_FOPoles(1:vit_CntrPar_WE_FOPoles_n_v2) = &
                vit_tmp_CntrPar_WE_FOPoles(1:vit_CntrPar_WE_FOPoles_n_v2)
        END IF
        vit_local_CntrPar%Y_ControlMode = vit_CntrPar_Y_ControlMode
        vit_local_CntrPar%Y_uSwitch = vit_CntrPar_Y_uSwitch
        IF (C_ASSOCIATED(vit_CntrPar_Y_ErrThresh_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%Y_ErrThresh(vit_CntrPar_Y_ErrThresh_n))
            CALL C_F_POINTER(vit_CntrPar_Y_ErrThresh_ptr, vit_tmp_CntrPar_Y_ErrThresh, [vit_CntrPar_Y_ErrThresh_n])
            vit_local_CntrPar%Y_ErrThresh(1:vit_CntrPar_Y_ErrThresh_n) = &
                vit_tmp_CntrPar_Y_ErrThresh(1:vit_CntrPar_Y_ErrThresh_n)
        END IF
        vit_local_CntrPar%Y_Rate = vit_CntrPar_Y_Rate
        vit_local_CntrPar%Y_MErrSet = vit_CntrPar_Y_MErrSet
        vit_local_CntrPar%Y_IPC_IntSat = vit_CntrPar_Y_IPC_IntSat
        vit_local_CntrPar%Y_IPC_KP = vit_CntrPar_Y_IPC_KP
        vit_local_CntrPar%Y_IPC_KI = vit_CntrPar_Y_IPC_KI
        vit_local_CntrPar%PS_Mode = vit_CntrPar_PS_Mode
        vit_local_CntrPar%PS_BldPitchMin_N = vit_CntrPar_PS_BldPitchMin_N
        IF (C_ASSOCIATED(vit_CntrPar_PS_WindSpeeds_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PS_WindSpeeds(vit_CntrPar_PS_WindSpeeds_n))
            CALL C_F_POINTER(vit_CntrPar_PS_WindSpeeds_ptr, vit_tmp_CntrPar_PS_WindSpeeds, &
                [vit_CntrPar_PS_WindSpeeds_n])
            vit_local_CntrPar%PS_WindSpeeds(1:vit_CntrPar_PS_WindSpeeds_n) = &
                vit_tmp_CntrPar_PS_WindSpeeds(1:vit_CntrPar_PS_WindSpeeds_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PS_BldPitchMin_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PS_BldPitchMin(vit_CntrPar_PS_BldPitchMin_n_v2))
            CALL C_F_POINTER(vit_CntrPar_PS_BldPitchMin_ptr, vit_tmp_CntrPar_PS_BldPitchMin, &
                [vit_CntrPar_PS_BldPitchMin_n_v2])
            vit_local_CntrPar%PS_BldPitchMin(1:vit_CntrPar_PS_BldPitchMin_n_v2) = &
                vit_tmp_CntrPar_PS_BldPitchMin(1:vit_CntrPar_PS_BldPitchMin_n_v2)
        END IF
        vit_local_CntrPar%SU_Mode = vit_CntrPar_SU_Mode
        vit_local_CntrPar%SU_StartTime = vit_CntrPar_SU_StartTime
        vit_local_CntrPar%SU_FW_MinDuration = vit_CntrPar_SU_FW_MinDuration
        vit_local_CntrPar%SU_RotorSpeedThresh = vit_CntrPar_SU_RotorSpeedThresh
        vit_local_CntrPar%SU_RotorSpeedCornerFreq = vit_CntrPar_SU_RotorSpeedCornerFreq
        vit_local_CntrPar%SU_LoadStages_N = vit_CntrPar_SU_LoadStages_N
        IF (C_ASSOCIATED(vit_CntrPar_SU_LoadStages_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SU_LoadStages(vit_CntrPar_SU_LoadStages_n_v2))
            CALL C_F_POINTER(vit_CntrPar_SU_LoadStages_ptr, vit_tmp_CntrPar_SU_LoadStages, &
                [vit_CntrPar_SU_LoadStages_n_v2])
            vit_local_CntrPar%SU_LoadStages(1:vit_CntrPar_SU_LoadStages_n_v2) = &
                vit_tmp_CntrPar_SU_LoadStages(1:vit_CntrPar_SU_LoadStages_n_v2)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SU_LoadRampDuration_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SU_LoadRampDuration(vit_CntrPar_SU_LoadRampDuration_n))
            CALL C_F_POINTER(vit_CntrPar_SU_LoadRampDuration_ptr, vit_tmp_CntrPar_SU_LoadRampDuration, &
                [vit_CntrPar_SU_LoadRampDuration_n])
            vit_local_CntrPar%SU_LoadRampDuration(1:vit_CntrPar_SU_LoadRampDuration_n) = &
                vit_tmp_CntrPar_SU_LoadRampDuration(1:vit_CntrPar_SU_LoadRampDuration_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SU_LoadHoldDuration_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SU_LoadHoldDuration(vit_CntrPar_SU_LoadHoldDuration_n))
            CALL C_F_POINTER(vit_CntrPar_SU_LoadHoldDuration_ptr, vit_tmp_CntrPar_SU_LoadHoldDuration, &
                [vit_CntrPar_SU_LoadHoldDuration_n])
            vit_local_CntrPar%SU_LoadHoldDuration(1:vit_CntrPar_SU_LoadHoldDuration_n) = &
                vit_tmp_CntrPar_SU_LoadHoldDuration(1:vit_CntrPar_SU_LoadHoldDuration_n)
        END IF
        vit_local_CntrPar%SD_Mode = vit_CntrPar_SD_Mode
        vit_local_CntrPar%SD_TimeActivate = vit_CntrPar_SD_TimeActivate
        vit_local_CntrPar%SD_EnablePitch = vit_CntrPar_SD_EnablePitch
        vit_local_CntrPar%SD_EnableYawError = vit_CntrPar_SD_EnableYawError
        vit_local_CntrPar%SD_EnableGenSpeed = vit_CntrPar_SD_EnableGenSpeed
        vit_local_CntrPar%SD_EnableTime = vit_CntrPar_SD_EnableTime
        vit_local_CntrPar%SD_MaxPit = vit_CntrPar_SD_MaxPit
        vit_local_CntrPar%SD_PitchCornerFreq = vit_CntrPar_SD_PitchCornerFreq
        vit_local_CntrPar%SD_MaxYawError = vit_CntrPar_SD_MaxYawError
        vit_local_CntrPar%SD_YawErrorCornerFreq = vit_CntrPar_SD_YawErrorCornerFreq
        vit_local_CntrPar%SD_MaxGenSpd = vit_CntrPar_SD_MaxGenSpd
        vit_local_CntrPar%SD_GenSpdCornerFreq = vit_CntrPar_SD_GenSpdCornerFreq
        vit_local_CntrPar%SD_Time = vit_CntrPar_SD_Time
        vit_local_CntrPar%SD_Method = vit_CntrPar_SD_Method
        IF (C_ASSOCIATED(vit_CntrPar_SD_MaxTorqueRate_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SD_MaxTorqueRate(vit_CntrPar_SD_MaxTorqueRate_n))
            CALL C_F_POINTER(vit_CntrPar_SD_MaxTorqueRate_ptr, vit_tmp_CntrPar_SD_MaxTorqueRate, &
                [vit_CntrPar_SD_MaxTorqueRate_n])
            vit_local_CntrPar%SD_MaxTorqueRate(1:vit_CntrPar_SD_MaxTorqueRate_n) = &
                vit_tmp_CntrPar_SD_MaxTorqueRate(1:vit_CntrPar_SD_MaxTorqueRate_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SD_MaxPitchRate_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SD_MaxPitchRate(vit_CntrPar_SD_MaxPitchRate_n))
            CALL C_F_POINTER(vit_CntrPar_SD_MaxPitchRate_ptr, vit_tmp_CntrPar_SD_MaxPitchRate, &
                [vit_CntrPar_SD_MaxPitchRate_n])
            vit_local_CntrPar%SD_MaxPitchRate(1:vit_CntrPar_SD_MaxPitchRate_n) = &
                vit_tmp_CntrPar_SD_MaxPitchRate(1:vit_CntrPar_SD_MaxPitchRate_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SD_StagePitch_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SD_StagePitch(vit_CntrPar_SD_StagePitch_n))
            CALL C_F_POINTER(vit_CntrPar_SD_StagePitch_ptr, vit_tmp_CntrPar_SD_StagePitch, &
                [vit_CntrPar_SD_StagePitch_n])
            vit_local_CntrPar%SD_StagePitch(1:vit_CntrPar_SD_StagePitch_n) = &
                vit_tmp_CntrPar_SD_StagePitch(1:vit_CntrPar_SD_StagePitch_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SD_StageTime_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%SD_StageTime(vit_CntrPar_SD_StageTime_n))
            CALL C_F_POINTER(vit_CntrPar_SD_StageTime_ptr, vit_tmp_CntrPar_SD_StageTime, [vit_CntrPar_SD_StageTime_n])
            vit_local_CntrPar%SD_StageTime(1:vit_CntrPar_SD_StageTime_n) = &
                vit_tmp_CntrPar_SD_StageTime(1:vit_CntrPar_SD_StageTime_n)
        END IF
        vit_local_CntrPar%SD_Stage_N = vit_CntrPar_SD_Stage_N
        vit_local_CntrPar%Fl_Mode = vit_CntrPar_Fl_Mode
        vit_local_CntrPar%Fl_n = vit_CntrPar_Fl_n
        IF (C_ASSOCIATED(vit_CntrPar_Fl_Kp_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%Fl_Kp(vit_CntrPar_Fl_Kp_n))
            CALL C_F_POINTER(vit_CntrPar_Fl_Kp_ptr, vit_tmp_CntrPar_Fl_Kp, [vit_CntrPar_Fl_Kp_n])
            vit_local_CntrPar%Fl_Kp(1:vit_CntrPar_Fl_Kp_n) = vit_tmp_CntrPar_Fl_Kp(1:vit_CntrPar_Fl_Kp_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_Fl_U_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%Fl_U(vit_CntrPar_Fl_U_n))
            CALL C_F_POINTER(vit_CntrPar_Fl_U_ptr, vit_tmp_CntrPar_Fl_U, [vit_CntrPar_Fl_U_n])
            vit_local_CntrPar%Fl_U(1:vit_CntrPar_Fl_U_n) = vit_tmp_CntrPar_Fl_U(1:vit_CntrPar_Fl_U_n)
        END IF
        vit_local_CntrPar%Flp_Mode = vit_CntrPar_Flp_Mode
        vit_local_CntrPar%Flp_Angle = vit_CntrPar_Flp_Angle
        vit_local_CntrPar%Flp_Kp = vit_CntrPar_Flp_Kp
        vit_local_CntrPar%Flp_Ki = vit_CntrPar_Flp_Ki
        vit_local_CntrPar%Flp_MaxPit = vit_CntrPar_Flp_MaxPit
        DO vit_ci_CntrPar_OL_Filename = 1, 1024
            vit_local_CntrPar%OL_Filename(vit_ci_CntrPar_OL_Filename:vit_ci_CntrPar_OL_Filename) = &
                vit_CntrPar_OL_Filename(vit_ci_CntrPar_OL_Filename)
        END DO
        vit_local_CntrPar%OL_Mode = vit_CntrPar_OL_Mode
        vit_local_CntrPar%OL_BP_Mode = vit_CntrPar_OL_BP_Mode
        vit_local_CntrPar%OL_BP_FiltFreq = vit_CntrPar_OL_BP_FiltFreq
        vit_local_CntrPar%Ind_Breakpoint = vit_CntrPar_Ind_Breakpoint
        IF (C_ASSOCIATED(vit_CntrPar_Ind_BldPitch_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%Ind_BldPitch(vit_CntrPar_Ind_BldPitch_n))
            CALL C_F_POINTER(vit_CntrPar_Ind_BldPitch_ptr, vit_tmp_CntrPar_Ind_BldPitch, [vit_CntrPar_Ind_BldPitch_n])
            vit_local_CntrPar%Ind_BldPitch(1:vit_CntrPar_Ind_BldPitch_n) = &
                vit_tmp_CntrPar_Ind_BldPitch(1:vit_CntrPar_Ind_BldPitch_n)
        END IF
        vit_local_CntrPar%Ind_GenTq = vit_CntrPar_Ind_GenTq
        vit_local_CntrPar%Ind_YawRate = vit_CntrPar_Ind_YawRate
        vit_local_CntrPar%Ind_R_Speed = vit_CntrPar_Ind_R_Speed
        vit_local_CntrPar%Ind_R_Torque = vit_CntrPar_Ind_R_Torque
        vit_local_CntrPar%Ind_R_Pitch = vit_CntrPar_Ind_R_Pitch
        vit_local_CntrPar%Ind_Azimuth = vit_CntrPar_Ind_Azimuth
        IF (C_ASSOCIATED(vit_CntrPar_RP_Gains_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%RP_Gains(vit_CntrPar_RP_Gains_n))
            CALL C_F_POINTER(vit_CntrPar_RP_Gains_ptr, vit_tmp_CntrPar_RP_Gains, [vit_CntrPar_RP_Gains_n])
            vit_local_CntrPar%RP_Gains(1:vit_CntrPar_RP_Gains_n) = vit_tmp_CntrPar_RP_Gains(1:vit_CntrPar_RP_Gains_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_Ind_CableControl_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%Ind_CableControl(vit_CntrPar_Ind_CableControl_n))
            CALL C_F_POINTER(vit_CntrPar_Ind_CableControl_ptr, vit_tmp_CntrPar_Ind_CableControl, &
                [vit_CntrPar_Ind_CableControl_n])
            vit_local_CntrPar%Ind_CableControl(1:vit_CntrPar_Ind_CableControl_n) = &
                vit_tmp_CntrPar_Ind_CableControl(1:vit_CntrPar_Ind_CableControl_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_Ind_StructControl_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%Ind_StructControl(vit_CntrPar_Ind_StructControl_n))
            CALL C_F_POINTER(vit_CntrPar_Ind_StructControl_ptr, vit_tmp_CntrPar_Ind_StructControl, &
                [vit_CntrPar_Ind_StructControl_n])
            vit_local_CntrPar%Ind_StructControl(1:vit_CntrPar_Ind_StructControl_n) = &
                vit_tmp_CntrPar_Ind_StructControl(1:vit_CntrPar_Ind_StructControl_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_Breakpoints_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_Breakpoints(vit_CntrPar_OL_Breakpoints_n))
            CALL C_F_POINTER(vit_CntrPar_OL_Breakpoints_ptr, vit_tmp_CntrPar_OL_Breakpoints, &
                [vit_CntrPar_OL_Breakpoints_n])
            vit_local_CntrPar%OL_Breakpoints(1:vit_CntrPar_OL_Breakpoints_n) = &
                vit_tmp_CntrPar_OL_Breakpoints(1:vit_CntrPar_OL_Breakpoints_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_BldPitch1_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_BldPitch1(vit_CntrPar_OL_BldPitch1_n))
            CALL C_F_POINTER(vit_CntrPar_OL_BldPitch1_ptr, vit_tmp_CntrPar_OL_BldPitch1, [vit_CntrPar_OL_BldPitch1_n])
            vit_local_CntrPar%OL_BldPitch1(1:vit_CntrPar_OL_BldPitch1_n) = &
                vit_tmp_CntrPar_OL_BldPitch1(1:vit_CntrPar_OL_BldPitch1_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_BldPitch2_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_BldPitch2(vit_CntrPar_OL_BldPitch2_n))
            CALL C_F_POINTER(vit_CntrPar_OL_BldPitch2_ptr, vit_tmp_CntrPar_OL_BldPitch2, [vit_CntrPar_OL_BldPitch2_n])
            vit_local_CntrPar%OL_BldPitch2(1:vit_CntrPar_OL_BldPitch2_n) = &
                vit_tmp_CntrPar_OL_BldPitch2(1:vit_CntrPar_OL_BldPitch2_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_BldPitch3_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_BldPitch3(vit_CntrPar_OL_BldPitch3_n))
            CALL C_F_POINTER(vit_CntrPar_OL_BldPitch3_ptr, vit_tmp_CntrPar_OL_BldPitch3, [vit_CntrPar_OL_BldPitch3_n])
            vit_local_CntrPar%OL_BldPitch3(1:vit_CntrPar_OL_BldPitch3_n) = &
                vit_tmp_CntrPar_OL_BldPitch3(1:vit_CntrPar_OL_BldPitch3_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_CableControl_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_CableControl(vit_CntrPar_OL_CableControl_rows, &
                vit_CntrPar_OL_CableControl_cols))
            CALL C_F_POINTER(vit_CntrPar_OL_CableControl_ptr, vit_tmp_CntrPar_OL_CableControl, &
                [vit_CntrPar_OL_CableControl_rows * vit_CntrPar_OL_CableControl_cols])
            DO vit_j_OL_CableControl = 1, vit_CntrPar_OL_CableControl_cols
                DO vit_i_OL_CableControl = 1, vit_CntrPar_OL_CableControl_rows
                    vit_local_CntrPar%OL_CableControl(vit_i_OL_CableControl, vit_j_OL_CableControl) = &
                        vit_tmp_CntrPar_OL_CableControl((vit_j_OL_CableControl-1)*vit_CntrPar_OL_CableControl_rows + &
                            vit_i_OL_CableControl)
                END DO
            END DO
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_StructControl_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_StructControl(vit_CntrPar_OL_StructControl_rows, &
                vit_CntrPar_OL_StructControl_cols))
            CALL C_F_POINTER(vit_CntrPar_OL_StructControl_ptr, vit_tmp_CntrPar_OL_StructControl, &
                [vit_CntrPar_OL_StructControl_rows * vit_CntrPar_OL_StructControl_cols])
            DO vit_j_OL_StructControl = 1, vit_CntrPar_OL_StructControl_cols
                DO vit_i_OL_StructControl = 1, vit_CntrPar_OL_StructControl_rows
                    vit_local_CntrPar%OL_StructControl(vit_i_OL_StructControl, vit_j_OL_StructControl) = &
                        vit_tmp_CntrPar_OL_StructControl((vit_j_OL_StructControl-1)*vit_CntrPar_OL_StructControl_rows &
                            + vit_i_OL_StructControl)
                END DO
            END DO
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_GenTq_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_GenTq(vit_CntrPar_OL_GenTq_n))
            CALL C_F_POINTER(vit_CntrPar_OL_GenTq_ptr, vit_tmp_CntrPar_OL_GenTq, [vit_CntrPar_OL_GenTq_n])
            vit_local_CntrPar%OL_GenTq(1:vit_CntrPar_OL_GenTq_n) = vit_tmp_CntrPar_OL_GenTq(1:vit_CntrPar_OL_GenTq_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_YawRate_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_YawRate(vit_CntrPar_OL_YawRate_n))
            CALL C_F_POINTER(vit_CntrPar_OL_YawRate_ptr, vit_tmp_CntrPar_OL_YawRate, [vit_CntrPar_OL_YawRate_n])
            vit_local_CntrPar%OL_YawRate(1:vit_CntrPar_OL_YawRate_n) = &
                vit_tmp_CntrPar_OL_YawRate(1:vit_CntrPar_OL_YawRate_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_Azimuth_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_Azimuth(vit_CntrPar_OL_Azimuth_n))
            CALL C_F_POINTER(vit_CntrPar_OL_Azimuth_ptr, vit_tmp_CntrPar_OL_Azimuth, [vit_CntrPar_OL_Azimuth_n])
            vit_local_CntrPar%OL_Azimuth(1:vit_CntrPar_OL_Azimuth_n) = &
                vit_tmp_CntrPar_OL_Azimuth(1:vit_CntrPar_OL_Azimuth_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_R_Speed_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_R_Speed(vit_CntrPar_OL_R_Speed_n))
            CALL C_F_POINTER(vit_CntrPar_OL_R_Speed_ptr, vit_tmp_CntrPar_OL_R_Speed, [vit_CntrPar_OL_R_Speed_n])
            vit_local_CntrPar%OL_R_Speed(1:vit_CntrPar_OL_R_Speed_n) = &
                vit_tmp_CntrPar_OL_R_Speed(1:vit_CntrPar_OL_R_Speed_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_R_Torque_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_R_Torque(vit_CntrPar_OL_R_Torque_n))
            CALL C_F_POINTER(vit_CntrPar_OL_R_Torque_ptr, vit_tmp_CntrPar_OL_R_Torque, [vit_CntrPar_OL_R_Torque_n])
            vit_local_CntrPar%OL_R_Torque(1:vit_CntrPar_OL_R_Torque_n) = &
                vit_tmp_CntrPar_OL_R_Torque(1:vit_CntrPar_OL_R_Torque_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_R_Pitch_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_R_Pitch(vit_CntrPar_OL_R_Pitch_n))
            CALL C_F_POINTER(vit_CntrPar_OL_R_Pitch_ptr, vit_tmp_CntrPar_OL_R_Pitch, [vit_CntrPar_OL_R_Pitch_n])
            vit_local_CntrPar%OL_R_Pitch(1:vit_CntrPar_OL_R_Pitch_n) = &
                vit_tmp_CntrPar_OL_R_Pitch(1:vit_CntrPar_OL_R_Pitch_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_Channels_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%OL_Channels(vit_CntrPar_OL_Channels_rows, vit_CntrPar_OL_Channels_cols))
            CALL C_F_POINTER(vit_CntrPar_OL_Channels_ptr, vit_tmp_CntrPar_OL_Channels, &
                [vit_CntrPar_OL_Channels_rows * vit_CntrPar_OL_Channels_cols])
            DO vit_j_OL_Channels = 1, vit_CntrPar_OL_Channels_cols
                DO vit_i_OL_Channels = 1, vit_CntrPar_OL_Channels_rows
                    vit_local_CntrPar%OL_Channels(vit_i_OL_Channels, vit_j_OL_Channels) = &
                        vit_tmp_CntrPar_OL_Channels((vit_j_OL_Channels-1)*vit_CntrPar_OL_Channels_rows + &
                            vit_i_OL_Channels)
                END DO
            END DO
        END IF
        vit_local_CntrPar%PA_Mode = vit_CntrPar_PA_Mode
        vit_local_CntrPar%PA_CornerFreq = vit_CntrPar_PA_CornerFreq
        vit_local_CntrPar%PA_Damping = vit_CntrPar_PA_Damping
        vit_local_CntrPar%AWC_Mode = vit_CntrPar_AWC_Mode
        vit_local_CntrPar%AWC_NumModes = vit_CntrPar_AWC_NumModes
        IF (C_ASSOCIATED(vit_CntrPar_AWC_n_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%AWC_n(vit_CntrPar_AWC_n_n))
            CALL C_F_POINTER(vit_CntrPar_AWC_n_ptr, vit_tmp_CntrPar_AWC_n, [vit_CntrPar_AWC_n_n])
            vit_local_CntrPar%AWC_n(1:vit_CntrPar_AWC_n_n) = vit_tmp_CntrPar_AWC_n(1:vit_CntrPar_AWC_n_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_harmonic_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%AWC_harmonic(vit_CntrPar_AWC_harmonic_n))
            CALL C_F_POINTER(vit_CntrPar_AWC_harmonic_ptr, vit_tmp_CntrPar_AWC_harmonic, [vit_CntrPar_AWC_harmonic_n])
            vit_local_CntrPar%AWC_harmonic(1:vit_CntrPar_AWC_harmonic_n) = &
                vit_tmp_CntrPar_AWC_harmonic(1:vit_CntrPar_AWC_harmonic_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_freq_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%AWC_freq(vit_CntrPar_AWC_freq_n))
            CALL C_F_POINTER(vit_CntrPar_AWC_freq_ptr, vit_tmp_CntrPar_AWC_freq, [vit_CntrPar_AWC_freq_n])
            vit_local_CntrPar%AWC_freq(1:vit_CntrPar_AWC_freq_n) = vit_tmp_CntrPar_AWC_freq(1:vit_CntrPar_AWC_freq_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_amp_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%AWC_amp(vit_CntrPar_AWC_amp_n))
            CALL C_F_POINTER(vit_CntrPar_AWC_amp_ptr, vit_tmp_CntrPar_AWC_amp, [vit_CntrPar_AWC_amp_n])
            vit_local_CntrPar%AWC_amp(1:vit_CntrPar_AWC_amp_n) = vit_tmp_CntrPar_AWC_amp(1:vit_CntrPar_AWC_amp_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_clockangle_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%AWC_clockangle(vit_CntrPar_AWC_clockangle_n))
            CALL C_F_POINTER(vit_CntrPar_AWC_clockangle_ptr, vit_tmp_CntrPar_AWC_clockangle, &
                [vit_CntrPar_AWC_clockangle_n])
            vit_local_CntrPar%AWC_clockangle(1:vit_CntrPar_AWC_clockangle_n) = &
                vit_tmp_CntrPar_AWC_clockangle(1:vit_CntrPar_AWC_clockangle_n)
        END IF
        vit_local_CntrPar%AWC_phaseoffset = vit_CntrPar_AWC_phaseoffset
        IF (C_ASSOCIATED(vit_CntrPar_AWC_CntrGains_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%AWC_CntrGains(vit_CntrPar_AWC_CntrGains_n))
            CALL C_F_POINTER(vit_CntrPar_AWC_CntrGains_ptr, vit_tmp_CntrPar_AWC_CntrGains, &
                [vit_CntrPar_AWC_CntrGains_n])
            vit_local_CntrPar%AWC_CntrGains(1:vit_CntrPar_AWC_CntrGains_n) = &
                vit_tmp_CntrPar_AWC_CntrGains(1:vit_CntrPar_AWC_CntrGains_n)
        END IF
        vit_local_CntrPar%PF_Mode = vit_CntrPar_PF_Mode
        IF (C_ASSOCIATED(vit_CntrPar_PF_Offsets_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PF_Offsets(vit_CntrPar_PF_Offsets_n))
            CALL C_F_POINTER(vit_CntrPar_PF_Offsets_ptr, vit_tmp_CntrPar_PF_Offsets, [vit_CntrPar_PF_Offsets_n])
            vit_local_CntrPar%PF_Offsets(1:vit_CntrPar_PF_Offsets_n) = &
                vit_tmp_CntrPar_PF_Offsets(1:vit_CntrPar_PF_Offsets_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PF_TimeStuck_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%PF_TimeStuck(vit_CntrPar_PF_TimeStuck_n))
            CALL C_F_POINTER(vit_CntrPar_PF_TimeStuck_ptr, vit_tmp_CntrPar_PF_TimeStuck, [vit_CntrPar_PF_TimeStuck_n])
            vit_local_CntrPar%PF_TimeStuck(1:vit_CntrPar_PF_TimeStuck_n) = &
                vit_tmp_CntrPar_PF_TimeStuck(1:vit_CntrPar_PF_TimeStuck_n)
        END IF
        vit_local_CntrPar%Ext_Mode = vit_CntrPar_Ext_Mode
        DO vit_ci_CntrPar_DLL_FileName = 1, 1024
            vit_local_CntrPar%DLL_FileName(vit_ci_CntrPar_DLL_FileName:vit_ci_CntrPar_DLL_FileName) = &
                vit_CntrPar_DLL_FileName(vit_ci_CntrPar_DLL_FileName)
        END DO
        DO vit_ci_CntrPar_DLL_InFile = 1, 1024
            vit_local_CntrPar%DLL_InFile(vit_ci_CntrPar_DLL_InFile:vit_ci_CntrPar_DLL_InFile) = &
                vit_CntrPar_DLL_InFile(vit_ci_CntrPar_DLL_InFile)
        END DO
        DO vit_ci_CntrPar_DLL_ProcName = 1, 1024
            vit_local_CntrPar%DLL_ProcName(vit_ci_CntrPar_DLL_ProcName:vit_ci_CntrPar_DLL_ProcName) = &
                vit_CntrPar_DLL_ProcName(vit_ci_CntrPar_DLL_ProcName)
        END DO
        vit_local_CntrPar%ZMQ_Mode = vit_CntrPar_ZMQ_Mode
        DO vit_ci_CntrPar_ZMQ_CommAddress = 1, 256
            vit_local_CntrPar%ZMQ_CommAddress(vit_ci_CntrPar_ZMQ_CommAddress:vit_ci_CntrPar_ZMQ_CommAddress) = &
                vit_CntrPar_ZMQ_CommAddress(vit_ci_CntrPar_ZMQ_CommAddress)
        END DO
        vit_local_CntrPar%ZMQ_UpdatePeriod = vit_CntrPar_ZMQ_UpdatePeriod
        vit_local_CntrPar%CC_Mode = vit_CntrPar_CC_Mode
        vit_local_CntrPar%CC_Group_N = vit_CntrPar_CC_Group_N
        vit_local_CntrPar%CC_ActTau = vit_CntrPar_CC_ActTau
        IF (C_ASSOCIATED(vit_CntrPar_CC_GroupIndex_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%CC_GroupIndex(vit_CntrPar_CC_GroupIndex_n))
            CALL C_F_POINTER(vit_CntrPar_CC_GroupIndex_ptr, vit_tmp_CntrPar_CC_GroupIndex, &
                [vit_CntrPar_CC_GroupIndex_n])
            vit_local_CntrPar%CC_GroupIndex(1:vit_CntrPar_CC_GroupIndex_n) = &
                vit_tmp_CntrPar_CC_GroupIndex(1:vit_CntrPar_CC_GroupIndex_n)
        END IF
        vit_local_CntrPar%StC_Mode = vit_CntrPar_StC_Mode
        vit_local_CntrPar%StC_Group_N = vit_CntrPar_StC_Group_N
        IF (C_ASSOCIATED(vit_CntrPar_StC_GroupIndex_ptr)) THEN
            ALLOCATE(vit_local_CntrPar%StC_GroupIndex(vit_CntrPar_StC_GroupIndex_n))
            CALL C_F_POINTER(vit_CntrPar_StC_GroupIndex_ptr, vit_tmp_CntrPar_StC_GroupIndex, &
                [vit_CntrPar_StC_GroupIndex_n])
            vit_local_CntrPar%StC_GroupIndex(1:vit_CntrPar_StC_GroupIndex_n) = &
                vit_tmp_CntrPar_StC_GroupIndex(1:vit_CntrPar_StC_GroupIndex_n)
        END IF
        vit_local_CntrPar%PC_RtTq99 = vit_CntrPar_PC_RtTq99
        vit_local_CntrPar%VS_MaxOMTq = vit_CntrPar_VS_MaxOMTq
        vit_local_CntrPar%VS_MinOMTq = vit_CntrPar_VS_MinOMTq
        vit_local_LocalVar%iStatus = vit_LocalVar_iStatus
        vit_local_LocalVar%AlreadyInitialized = vit_LocalVar_AlreadyInitialized
        vit_local_LocalVar%RestartWSE = vit_LocalVar_RestartWSE
        vit_local_LocalVar%Time = vit_LocalVar_Time
        vit_local_LocalVar%DT = vit_LocalVar_DT
        vit_local_LocalVar%WriteThisStep = vit_LocalVar_WriteThisStep
        vit_local_LocalVar%n_DT = vit_LocalVar_n_DT
        vit_local_LocalVar%Time_Last = vit_LocalVar_Time_Last
        vit_local_LocalVar%VS_GenPwr = vit_LocalVar_VS_GenPwr
        vit_local_LocalVar%GenSpeed = vit_LocalVar_GenSpeed
        vit_local_LocalVar%RotSpeed = vit_LocalVar_RotSpeed
        vit_local_LocalVar%NacHeading = vit_LocalVar_NacHeading
        vit_local_LocalVar%NacVane = vit_LocalVar_NacVane
        vit_local_LocalVar%NacVaneF = vit_LocalVar_NacVaneF
        vit_local_LocalVar%WindDir = vit_LocalVar_WindDir
        vit_local_LocalVar%HorWindV = vit_LocalVar_HorWindV
        vit_local_LocalVar%HorWindV_F = vit_LocalVar_HorWindV_F
        vit_local_LocalVar%rootMOOP = vit_LocalVar_rootMOOP
        vit_local_LocalVar%rootMOOPF = vit_LocalVar_rootMOOPF
        vit_local_LocalVar%BlPitch = vit_LocalVar_BlPitch
        vit_local_LocalVar%BlPitchCMeas = vit_LocalVar_BlPitchCMeas
        vit_local_LocalVar%Azimuth = vit_LocalVar_Azimuth
        vit_local_LocalVar%OL_Azimuth = vit_LocalVar_OL_Azimuth
        vit_local_LocalVar%AzUnwrapped = vit_LocalVar_AzUnwrapped
        vit_local_LocalVar%AzError = vit_LocalVar_AzError
        vit_local_LocalVar%GenTqAz = vit_LocalVar_GenTqAz
        vit_local_LocalVar%AzBuffer = vit_LocalVar_AzBuffer
        vit_local_LocalVar%NumBl = vit_LocalVar_NumBl
        vit_local_LocalVar%FA_Acc_TT = vit_LocalVar_FA_Acc_TT
        vit_local_LocalVar%SS_Acc_TT = vit_LocalVar_SS_Acc_TT
        vit_local_LocalVar%FA_Acc_Nac = vit_LocalVar_FA_Acc_Nac
        vit_local_LocalVar%NacIMU_FA_RAcc = vit_LocalVar_NacIMU_FA_RAcc
        vit_local_LocalVar%FA_AccHPF = vit_LocalVar_FA_AccHPF
        vit_local_LocalVar%FA_AccHPFI = vit_LocalVar_FA_AccHPFI
        vit_local_LocalVar%FA_PitCom = vit_LocalVar_FA_PitCom
        vit_local_LocalVar%VS_RefSpd = vit_LocalVar_VS_RefSpd
        vit_local_LocalVar%VS_RefSpd_TSR = vit_LocalVar_VS_RefSpd_TSR
        vit_local_LocalVar%VS_RefSpd_TRA = vit_LocalVar_VS_RefSpd_TRA
        vit_local_LocalVar%VS_RefSpd_RL = vit_LocalVar_VS_RefSpd_RL
        vit_local_LocalVar%PC_RefSpd = vit_LocalVar_PC_RefSpd
        vit_local_LocalVar%PC_RefSpd_SS = vit_LocalVar_PC_RefSpd_SS
        vit_local_LocalVar%PC_RefSpd_PRC = vit_LocalVar_PC_RefSpd_PRC
        vit_local_LocalVar%RotSpeedF = vit_LocalVar_RotSpeedF
        vit_local_LocalVar%GenSpeedF = vit_LocalVar_GenSpeedF
        vit_local_LocalVar%GenTq = vit_LocalVar_GenTq
        vit_local_LocalVar%GenTqMeas = vit_LocalVar_GenTqMeas
        vit_local_LocalVar%GenArTq = vit_LocalVar_GenArTq
        vit_local_LocalVar%GenBrTq = vit_LocalVar_GenBrTq
        vit_local_LocalVar%VS_KOmega2_GenTq = vit_LocalVar_VS_KOmega2_GenTq
        vit_local_LocalVar%VS_ConstPwr_GenTq = vit_LocalVar_VS_ConstPwr_GenTq
        vit_local_LocalVar%IPC_PitComF = vit_LocalVar_IPC_PitComF
        vit_local_LocalVar%PC_KP = vit_LocalVar_PC_KP
        vit_local_LocalVar%PC_KI = vit_LocalVar_PC_KI
        vit_local_LocalVar%PC_KD = vit_LocalVar_PC_KD
        vit_local_LocalVar%PC_TF = vit_LocalVar_PC_TF
        vit_local_LocalVar%PC_MaxPit = vit_LocalVar_PC_MaxPit
        vit_local_LocalVar%PC_MinPit = vit_LocalVar_PC_MinPit
        vit_local_LocalVar%PC_PitComT = vit_LocalVar_PC_PitComT
        vit_local_LocalVar%PC_PitComT_Last = vit_LocalVar_PC_PitComT_Last
        vit_local_LocalVar%BlPitchCMeasF = vit_LocalVar_BlPitchCMeasF
        vit_local_LocalVar%PC_PitComT_IPC = vit_LocalVar_PC_PitComT_IPC
        vit_local_LocalVar%PC_PwrErr = vit_LocalVar_PC_PwrErr
        vit_local_LocalVar%PC_SpdErr = vit_LocalVar_PC_SpdErr
        vit_local_LocalVar%IPC_AxisTilt_1P = vit_LocalVar_IPC_AxisTilt_1P
        vit_local_LocalVar%IPC_AxisYaw_1P = vit_LocalVar_IPC_AxisYaw_1P
        vit_local_LocalVar%IPC_AxisTilt_2P = vit_LocalVar_IPC_AxisTilt_2P
        vit_local_LocalVar%IPC_AxisYaw_2P = vit_LocalVar_IPC_AxisYaw_2P
        vit_local_LocalVar%axisTilt_1P = vit_LocalVar_axisTilt_1P
        vit_local_LocalVar%axisYaw_1P = vit_LocalVar_axisYaw_1P
        vit_local_LocalVar%axisYawF_1P = vit_LocalVar_axisYawF_1P
        vit_local_LocalVar%axisTilt_2P = vit_LocalVar_axisTilt_2P
        vit_local_LocalVar%axisYaw_2P = vit_LocalVar_axisYaw_2P
        vit_local_LocalVar%axisYawF_2P = vit_LocalVar_axisYawF_2P
        vit_local_LocalVar%IPC_KI = vit_LocalVar_IPC_KI
        vit_local_LocalVar%IPC_KP = vit_LocalVar_IPC_KP
        vit_local_LocalVar%IPC_IntSat = vit_LocalVar_IPC_IntSat
        vit_local_LocalVar%PC_State = vit_LocalVar_PC_State
        vit_local_LocalVar%PitCom = vit_LocalVar_PitCom
        vit_local_LocalVar%PitCom_SD = vit_LocalVar_PitCom_SD
        vit_local_LocalVar%PitComAct = vit_LocalVar_PitComAct
        vit_local_LocalVar%SS_DelOmegaF = vit_LocalVar_SS_DelOmegaF
        vit_local_LocalVar%TestType = vit_LocalVar_TestType
        vit_local_LocalVar%Kp_Float = vit_LocalVar_Kp_Float
        vit_local_LocalVar%VS_MaxTq = vit_LocalVar_VS_MaxTq
        vit_local_LocalVar%VS_LastGenTrq = vit_LocalVar_VS_LastGenTrq
        vit_local_LocalVar%VS_LastGenPwr = vit_LocalVar_VS_LastGenPwr
        vit_local_LocalVar%VS_MechGenPwr = vit_LocalVar_VS_MechGenPwr
        vit_local_LocalVar%VS_SpdErrAr = vit_LocalVar_VS_SpdErrAr
        vit_local_LocalVar%VS_SpdErrBr = vit_LocalVar_VS_SpdErrBr
        vit_local_LocalVar%VS_SpdErr = vit_LocalVar_VS_SpdErr
        vit_local_LocalVar%VS_State = vit_LocalVar_VS_State
        vit_local_LocalVar%VS_Rgn3Pitch = vit_LocalVar_VS_Rgn3Pitch
        vit_local_LocalVar%WE_Vw = vit_LocalVar_WE_Vw
        vit_local_LocalVar%WE_Vw_F = vit_LocalVar_WE_Vw_F
        vit_local_LocalVar%WE_VwI = vit_LocalVar_WE_VwI
        vit_local_LocalVar%WE_VwIdot = vit_LocalVar_WE_VwIdot
        vit_local_LocalVar%WE_Op = vit_LocalVar_WE_Op
        vit_local_LocalVar%WE_Op_Last = vit_LocalVar_WE_Op_Last
        vit_local_LocalVar%VS_LastGenTrqF = vit_LocalVar_VS_LastGenTrqF
        vit_local_LocalVar%PRC_WSE_F = vit_LocalVar_PRC_WSE_F
        vit_local_LocalVar%PRC_R_Speed = vit_LocalVar_PRC_R_Speed
        vit_local_LocalVar%PRC_R_Torque = vit_LocalVar_PRC_R_Torque
        vit_local_LocalVar%PRC_R_Pitch = vit_LocalVar_PRC_R_Pitch
        vit_local_LocalVar%PRC_R_Total = vit_LocalVar_PRC_R_Total
        vit_local_LocalVar%PRC_Min_Pitch = vit_LocalVar_PRC_Min_Pitch
        vit_local_LocalVar%PS_Min_Pitch = vit_LocalVar_PS_Min_Pitch
        vit_local_LocalVar%OL_Index = vit_LocalVar_OL_Index
        vit_local_LocalVar%SU_Stage = vit_LocalVar_SU_Stage
        vit_local_LocalVar%SU_LoadStageStartTime = vit_LocalVar_SU_LoadStageStartTime
        vit_local_LocalVar%SU_RotSpeedF = vit_LocalVar_SU_RotSpeedF
        vit_local_LocalVar%SD_Trigger = vit_LocalVar_SD_Trigger
        vit_local_LocalVar%SD_BlPitchF = vit_LocalVar_SD_BlPitchF
        vit_local_LocalVar%SD_NacVaneF = vit_LocalVar_SD_NacVaneF
        vit_local_LocalVar%SD_GenSpeedF = vit_LocalVar_SD_GenSpeedF
        vit_local_LocalVar%SD_Stage = vit_LocalVar_SD_Stage
        vit_local_LocalVar%SD_StageStartTime = vit_LocalVar_SD_StageStartTime
        vit_local_LocalVar%SD_MaxPitchRate = vit_LocalVar_SD_MaxPitchRate
        vit_local_LocalVar%SD_MaxTorqueRate = vit_LocalVar_SD_MaxTorqueRate
        vit_local_LocalVar%GenTq_SD = vit_LocalVar_GenTq_SD
        vit_local_LocalVar%Fl_PitCom = vit_LocalVar_Fl_PitCom
        vit_local_LocalVar%NACIMU_FA_AccF = vit_LocalVar_NACIMU_FA_AccF
        vit_local_LocalVar%FA_AccF = vit_LocalVar_FA_AccF
        vit_local_LocalVar%FA_Hist = vit_LocalVar_FA_Hist
        vit_local_LocalVar%TRA_LastRefSpd = vit_LocalVar_TRA_LastRefSpd
        vit_local_LocalVar%VS_RefSpeed = vit_LocalVar_VS_RefSpeed
        vit_local_LocalVar%PtfmTDX = vit_LocalVar_PtfmTDX
        vit_local_LocalVar%PtfmTDY = vit_LocalVar_PtfmTDY
        vit_local_LocalVar%PtfmTDZ = vit_LocalVar_PtfmTDZ
        vit_local_LocalVar%PtfmRDX = vit_LocalVar_PtfmRDX
        vit_local_LocalVar%PtfmRDY = vit_LocalVar_PtfmRDY
        vit_local_LocalVar%PtfmRDZ = vit_LocalVar_PtfmRDZ
        vit_local_LocalVar%PtfmTVX = vit_LocalVar_PtfmTVX
        vit_local_LocalVar%PtfmTVY = vit_LocalVar_PtfmTVY
        vit_local_LocalVar%PtfmTVZ = vit_LocalVar_PtfmTVZ
        vit_local_LocalVar%PtfmRVX = vit_LocalVar_PtfmRVX
        vit_local_LocalVar%PtfmRVY = vit_LocalVar_PtfmRVY
        vit_local_LocalVar%PtfmRVZ = vit_LocalVar_PtfmRVZ
        vit_local_LocalVar%PtfmTAX = vit_LocalVar_PtfmTAX
        vit_local_LocalVar%PtfmTAY = vit_LocalVar_PtfmTAY
        vit_local_LocalVar%PtfmTAZ = vit_LocalVar_PtfmTAZ
        vit_local_LocalVar%PtfmRAX = vit_LocalVar_PtfmRAX
        vit_local_LocalVar%PtfmRAY = vit_LocalVar_PtfmRAY
        vit_local_LocalVar%PtfmRAZ = vit_LocalVar_PtfmRAZ
        vit_local_LocalVar%CC_DesiredL = vit_LocalVar_CC_DesiredL
        vit_local_LocalVar%CC_ActuatedL = vit_LocalVar_CC_ActuatedL
        vit_local_LocalVar%CC_ActuatedDL = vit_LocalVar_CC_ActuatedDL
        vit_local_LocalVar%StC_Input = vit_LocalVar_StC_Input
        vit_local_LocalVar%Flp_Angle = vit_LocalVar_Flp_Angle
        vit_local_LocalVar%RootMyb_Last = vit_LocalVar_RootMyb_Last
        vit_local_LocalVar%ACC_INFILE_SIZE = vit_LocalVar_ACC_INFILE_SIZE
        IF (C_ASSOCIATED(vit_LocalVar_ACC_INFILE_ptr)) THEN
            ALLOCATE(vit_local_LocalVar%ACC_INFILE(vit_LocalVar_ACC_INFILE_n))
            CALL C_F_POINTER(vit_LocalVar_ACC_INFILE_ptr, vit_tmp_LocalVar_ACC_INFILE, [vit_LocalVar_ACC_INFILE_n])
            vit_local_LocalVar%ACC_INFILE(1:vit_LocalVar_ACC_INFILE_n) = &
                vit_tmp_LocalVar_ACC_INFILE(1:vit_LocalVar_ACC_INFILE_n)
        END IF
        vit_local_LocalVar%restart = vit_LocalVar_restart
        vit_local_LocalVar%AWC_complexangle = vit_LocalVar_AWC_complexangle
        vit_local_LocalVar%TiltMean = vit_LocalVar_TiltMean
        vit_local_LocalVar%YawMean = vit_LocalVar_YawMean
        vit_local_LocalVar%ZMQ_ID = vit_LocalVar_ZMQ_ID
        vit_local_LocalVar%ZMQ_YawOffset = vit_LocalVar_ZMQ_YawOffset
        vit_local_LocalVar%ZMQ_TorqueOffset = vit_LocalVar_ZMQ_TorqueOffset
        vit_local_LocalVar%ZMQ_PitOffset = vit_LocalVar_ZMQ_PitOffset
        vit_local_LocalVar%ZMQ_R_Speed = vit_LocalVar_ZMQ_R_Speed
        vit_local_LocalVar%ZMQ_R_Torque = vit_LocalVar_ZMQ_R_Torque
        vit_local_LocalVar%ZMQ_R_Pitch = vit_LocalVar_ZMQ_R_Pitch
        IF (C_ASSOCIATED(vit_LocalVar_WE_ptr)) THEN
            CALL C_F_POINTER(vit_LocalVar_WE_ptr, vit_nested_LocalVar_WE)
            vit_local_LocalVar%WE = vit_nested_LocalVar_WE
        END IF
        IF (C_ASSOCIATED(vit_LocalVar_FP_ptr)) THEN
            CALL C_F_POINTER(vit_LocalVar_FP_ptr, vit_nested_LocalVar_FP)
            vit_local_LocalVar%FP = vit_nested_LocalVar_FP
        END IF
        IF (C_ASSOCIATED(vit_LocalVar_piP_ptr)) THEN
            CALL C_F_POINTER(vit_LocalVar_piP_ptr, vit_nested_LocalVar_piP)
            vit_local_LocalVar%piP = vit_nested_LocalVar_piP
        END IF
        IF (C_ASSOCIATED(vit_LocalVar_resP_ptr)) THEN
            CALL C_F_POINTER(vit_LocalVar_resP_ptr, vit_nested_LocalVar_resP)
            vit_local_LocalVar%resP = vit_nested_LocalVar_resP
        END IF
        IF (C_ASSOCIATED(vit_LocalVar_rlP_ptr)) THEN
            CALL C_F_POINTER(vit_LocalVar_rlP_ptr, vit_nested_LocalVar_rlP)
            vit_local_LocalVar%rlP = vit_nested_LocalVar_rlP
        END IF
        IF (C_ASSOCIATED(vit_ExtDLL_avrSWAP_ptr)) THEN
            ALLOCATE(vit_local_ExtDLL%avrSWAP(vit_ExtDLL_avrSWAP_n))
            CALL C_F_POINTER(vit_ExtDLL_avrSWAP_ptr, vit_tmp_ExtDLL_avrSWAP, [vit_ExtDLL_avrSWAP_n])
            vit_local_ExtDLL%avrSWAP(1:vit_ExtDLL_avrSWAP_n) = vit_tmp_ExtDLL_avrSWAP(1:vit_ExtDLL_avrSWAP_n)
        END IF
        vit_local_ErrVar%size_avcMSG = vit_ErrVar_size_avcMSG
        vit_local_ErrVar%aviFAIL = vit_ErrVar_aviFAIL
        vit_local_ErrVar%ErrStat = vit_ErrVar_ErrStat
        IF (vit_ErrVar_ErrMsg_cap > 0) THEN
            ALLOCATE(CHARACTER(LEN=vit_ErrVar_ErrMsg_n) :: vit_local_ErrVar%ErrMsg)
            DO vit_dc_ErrVar_ErrMsg = 1, vit_ErrVar_ErrMsg_n
                vit_local_ErrVar%ErrMsg(vit_dc_ErrVar_ErrMsg:vit_dc_ErrVar_ErrMsg) = &
                    vit_ErrVar_ErrMsg(vit_dc_ErrVar_ErrMsg)
            END DO
        END IF
        CALL ExtController(avrSWAP, vit_local_CntrPar, vit_local_LocalVar, vit_local_ExtDLL, vit_local_ErrVar)
        DO vit_ci_CntrPar_PerfFileName = 1, 1024
            vit_CntrPar_PerfFileName(vit_ci_CntrPar_PerfFileName) = &
                vit_local_CntrPar%PerfFileName(vit_ci_CntrPar_PerfFileName:vit_ci_CntrPar_PerfFileName)
        END DO
        DO vit_ci_CntrPar_OL_Filename = 1, 1024
            vit_CntrPar_OL_Filename(vit_ci_CntrPar_OL_Filename) = &
                vit_local_CntrPar%OL_Filename(vit_ci_CntrPar_OL_Filename:vit_ci_CntrPar_OL_Filename)
        END DO
        DO vit_ci_CntrPar_DLL_FileName = 1, 1024
            vit_CntrPar_DLL_FileName(vit_ci_CntrPar_DLL_FileName) = &
                vit_local_CntrPar%DLL_FileName(vit_ci_CntrPar_DLL_FileName:vit_ci_CntrPar_DLL_FileName)
        END DO
        DO vit_ci_CntrPar_DLL_InFile = 1, 1024
            vit_CntrPar_DLL_InFile(vit_ci_CntrPar_DLL_InFile) = &
                vit_local_CntrPar%DLL_InFile(vit_ci_CntrPar_DLL_InFile:vit_ci_CntrPar_DLL_InFile)
        END DO
        DO vit_ci_CntrPar_DLL_ProcName = 1, 1024
            vit_CntrPar_DLL_ProcName(vit_ci_CntrPar_DLL_ProcName) = &
                vit_local_CntrPar%DLL_ProcName(vit_ci_CntrPar_DLL_ProcName:vit_ci_CntrPar_DLL_ProcName)
        END DO
        DO vit_ci_CntrPar_ZMQ_CommAddress = 1, 256
            vit_CntrPar_ZMQ_CommAddress(vit_ci_CntrPar_ZMQ_CommAddress) = &
                vit_local_CntrPar%ZMQ_CommAddress(vit_ci_CntrPar_ZMQ_CommAddress:vit_ci_CntrPar_ZMQ_CommAddress)
        END DO
        IF (ALLOCATED(vit_local_CntrPar%F_GenSpdNotch_Ind)) DEALLOCATE(vit_local_CntrPar%F_GenSpdNotch_Ind)
        IF (ALLOCATED(vit_local_CntrPar%F_TwrTopNotch_Ind)) DEALLOCATE(vit_local_CntrPar%F_TwrTopNotch_Ind)
        IF (ALLOCATED(vit_local_CntrPar%F_NotchFreqs)) DEALLOCATE(vit_local_CntrPar%F_NotchFreqs)
        IF (ALLOCATED(vit_local_CntrPar%F_NotchBetaNum)) DEALLOCATE(vit_local_CntrPar%F_NotchBetaNum)
        IF (ALLOCATED(vit_local_CntrPar%F_NotchBetaDen)) DEALLOCATE(vit_local_CntrPar%F_NotchBetaDen)
        IF (ALLOCATED(vit_local_CntrPar%F_FlCornerFreq)) DEALLOCATE(vit_local_CntrPar%F_FlCornerFreq)
        IF (ALLOCATED(vit_local_CntrPar%F_FlpCornerFreq)) DEALLOCATE(vit_local_CntrPar%F_FlpCornerFreq)
        IF (ALLOCATED(vit_local_CntrPar%IPC_Vramp)) DEALLOCATE(vit_local_CntrPar%IPC_Vramp)
        IF (ALLOCATED(vit_local_CntrPar%IPC_KP)) DEALLOCATE(vit_local_CntrPar%IPC_KP)
        IF (ALLOCATED(vit_local_CntrPar%IPC_KI)) DEALLOCATE(vit_local_CntrPar%IPC_KI)
        IF (ALLOCATED(vit_local_CntrPar%IPC_aziOffset)) DEALLOCATE(vit_local_CntrPar%IPC_aziOffset)
        IF (ALLOCATED(vit_local_CntrPar%PC_GS_angles)) DEALLOCATE(vit_local_CntrPar%PC_GS_angles)
        IF (ALLOCATED(vit_local_CntrPar%PC_GS_KP)) DEALLOCATE(vit_local_CntrPar%PC_GS_KP)
        IF (ALLOCATED(vit_local_CntrPar%PC_GS_KI)) DEALLOCATE(vit_local_CntrPar%PC_GS_KI)
        IF (ALLOCATED(vit_local_CntrPar%PC_GS_KD)) DEALLOCATE(vit_local_CntrPar%PC_GS_KD)
        IF (ALLOCATED(vit_local_CntrPar%PC_GS_TF)) DEALLOCATE(vit_local_CntrPar%PC_GS_TF)
        IF (ALLOCATED(vit_local_CntrPar%VS_KP)) DEALLOCATE(vit_local_CntrPar%VS_KP)
        IF (ALLOCATED(vit_local_CntrPar%VS_KI)) DEALLOCATE(vit_local_CntrPar%VS_KI)
        IF (ALLOCATED(vit_local_CntrPar%VS_FBP_U)) DEALLOCATE(vit_local_CntrPar%VS_FBP_U)
        IF (ALLOCATED(vit_local_CntrPar%VS_FBP_Omega)) DEALLOCATE(vit_local_CntrPar%VS_FBP_Omega)
        IF (ALLOCATED(vit_local_CntrPar%VS_FBP_Tau)) DEALLOCATE(vit_local_CntrPar%VS_FBP_Tau)
        IF (ALLOCATED(vit_local_CntrPar%PRC_WindSpeeds)) DEALLOCATE(vit_local_CntrPar%PRC_WindSpeeds)
        IF (ALLOCATED(vit_local_CntrPar%PRC_GenSpeeds)) DEALLOCATE(vit_local_CntrPar%PRC_GenSpeeds)
        IF (ALLOCATED(vit_local_CntrPar%PRC_Pitch_Table)) DEALLOCATE(vit_local_CntrPar%PRC_Pitch_Table)
        IF (ALLOCATED(vit_local_CntrPar%PRC_R_Table)) DEALLOCATE(vit_local_CntrPar%PRC_R_Table)
        IF (ALLOCATED(vit_local_CntrPar%WE_CP)) DEALLOCATE(vit_local_CntrPar%WE_CP)
        IF (ALLOCATED(vit_local_CntrPar%PerfTableSize)) DEALLOCATE(vit_local_CntrPar%PerfTableSize)
        IF (ALLOCATED(vit_local_CntrPar%WE_FOPoles_v)) DEALLOCATE(vit_local_CntrPar%WE_FOPoles_v)
        IF (ALLOCATED(vit_local_CntrPar%WE_FOPoles)) DEALLOCATE(vit_local_CntrPar%WE_FOPoles)
        IF (ALLOCATED(vit_local_CntrPar%Y_ErrThresh)) DEALLOCATE(vit_local_CntrPar%Y_ErrThresh)
        IF (ALLOCATED(vit_local_CntrPar%PS_WindSpeeds)) DEALLOCATE(vit_local_CntrPar%PS_WindSpeeds)
        IF (ALLOCATED(vit_local_CntrPar%PS_BldPitchMin)) DEALLOCATE(vit_local_CntrPar%PS_BldPitchMin)
        IF (ALLOCATED(vit_local_CntrPar%SU_LoadStages)) DEALLOCATE(vit_local_CntrPar%SU_LoadStages)
        IF (ALLOCATED(vit_local_CntrPar%SU_LoadRampDuration)) DEALLOCATE(vit_local_CntrPar%SU_LoadRampDuration)
        IF (ALLOCATED(vit_local_CntrPar%SU_LoadHoldDuration)) DEALLOCATE(vit_local_CntrPar%SU_LoadHoldDuration)
        IF (ALLOCATED(vit_local_CntrPar%SD_MaxTorqueRate)) DEALLOCATE(vit_local_CntrPar%SD_MaxTorqueRate)
        IF (ALLOCATED(vit_local_CntrPar%SD_MaxPitchRate)) DEALLOCATE(vit_local_CntrPar%SD_MaxPitchRate)
        IF (ALLOCATED(vit_local_CntrPar%SD_StagePitch)) DEALLOCATE(vit_local_CntrPar%SD_StagePitch)
        IF (ALLOCATED(vit_local_CntrPar%SD_StageTime)) DEALLOCATE(vit_local_CntrPar%SD_StageTime)
        IF (ALLOCATED(vit_local_CntrPar%Fl_Kp)) DEALLOCATE(vit_local_CntrPar%Fl_Kp)
        IF (ALLOCATED(vit_local_CntrPar%Fl_U)) DEALLOCATE(vit_local_CntrPar%Fl_U)
        IF (ALLOCATED(vit_local_CntrPar%Ind_BldPitch)) DEALLOCATE(vit_local_CntrPar%Ind_BldPitch)
        IF (ALLOCATED(vit_local_CntrPar%RP_Gains)) DEALLOCATE(vit_local_CntrPar%RP_Gains)
        IF (ALLOCATED(vit_local_CntrPar%Ind_CableControl)) DEALLOCATE(vit_local_CntrPar%Ind_CableControl)
        IF (ALLOCATED(vit_local_CntrPar%Ind_StructControl)) DEALLOCATE(vit_local_CntrPar%Ind_StructControl)
        IF (ALLOCATED(vit_local_CntrPar%OL_Breakpoints)) DEALLOCATE(vit_local_CntrPar%OL_Breakpoints)
        IF (ALLOCATED(vit_local_CntrPar%OL_BldPitch1)) DEALLOCATE(vit_local_CntrPar%OL_BldPitch1)
        IF (ALLOCATED(vit_local_CntrPar%OL_BldPitch2)) DEALLOCATE(vit_local_CntrPar%OL_BldPitch2)
        IF (ALLOCATED(vit_local_CntrPar%OL_BldPitch3)) DEALLOCATE(vit_local_CntrPar%OL_BldPitch3)
        IF (ALLOCATED(vit_local_CntrPar%OL_CableControl)) DEALLOCATE(vit_local_CntrPar%OL_CableControl)
        IF (ALLOCATED(vit_local_CntrPar%OL_StructControl)) DEALLOCATE(vit_local_CntrPar%OL_StructControl)
        IF (ALLOCATED(vit_local_CntrPar%OL_GenTq)) DEALLOCATE(vit_local_CntrPar%OL_GenTq)
        IF (ALLOCATED(vit_local_CntrPar%OL_YawRate)) DEALLOCATE(vit_local_CntrPar%OL_YawRate)
        IF (ALLOCATED(vit_local_CntrPar%OL_Azimuth)) DEALLOCATE(vit_local_CntrPar%OL_Azimuth)
        IF (ALLOCATED(vit_local_CntrPar%OL_R_Speed)) DEALLOCATE(vit_local_CntrPar%OL_R_Speed)
        IF (ALLOCATED(vit_local_CntrPar%OL_R_Torque)) DEALLOCATE(vit_local_CntrPar%OL_R_Torque)
        IF (ALLOCATED(vit_local_CntrPar%OL_R_Pitch)) DEALLOCATE(vit_local_CntrPar%OL_R_Pitch)
        IF (ALLOCATED(vit_local_CntrPar%OL_Channels)) DEALLOCATE(vit_local_CntrPar%OL_Channels)
        IF (ALLOCATED(vit_local_CntrPar%AWC_n)) DEALLOCATE(vit_local_CntrPar%AWC_n)
        IF (ALLOCATED(vit_local_CntrPar%AWC_harmonic)) DEALLOCATE(vit_local_CntrPar%AWC_harmonic)
        IF (ALLOCATED(vit_local_CntrPar%AWC_freq)) DEALLOCATE(vit_local_CntrPar%AWC_freq)
        IF (ALLOCATED(vit_local_CntrPar%AWC_amp)) DEALLOCATE(vit_local_CntrPar%AWC_amp)
        IF (ALLOCATED(vit_local_CntrPar%AWC_clockangle)) DEALLOCATE(vit_local_CntrPar%AWC_clockangle)
        IF (ALLOCATED(vit_local_CntrPar%AWC_CntrGains)) DEALLOCATE(vit_local_CntrPar%AWC_CntrGains)
        IF (ALLOCATED(vit_local_CntrPar%PF_Offsets)) DEALLOCATE(vit_local_CntrPar%PF_Offsets)
        IF (ALLOCATED(vit_local_CntrPar%PF_TimeStuck)) DEALLOCATE(vit_local_CntrPar%PF_TimeStuck)
        IF (ALLOCATED(vit_local_CntrPar%CC_GroupIndex)) DEALLOCATE(vit_local_CntrPar%CC_GroupIndex)
        IF (ALLOCATED(vit_local_CntrPar%StC_GroupIndex)) DEALLOCATE(vit_local_CntrPar%StC_GroupIndex)
        vit_LocalVar_rootMOOP = vit_local_LocalVar%rootMOOP
        vit_LocalVar_rootMOOPF = vit_local_LocalVar%rootMOOPF
        vit_LocalVar_BlPitch = vit_local_LocalVar%BlPitch
        vit_LocalVar_AzBuffer = vit_local_LocalVar%AzBuffer
        vit_LocalVar_FA_PitCom = vit_local_LocalVar%FA_PitCom
        vit_LocalVar_IPC_PitComF = vit_local_LocalVar%IPC_PitComF
        vit_LocalVar_PC_PitComT_IPC = vit_local_LocalVar%PC_PitComT_IPC
        vit_LocalVar_IPC_KI = vit_local_LocalVar%IPC_KI
        vit_LocalVar_IPC_KP = vit_local_LocalVar%IPC_KP
        vit_LocalVar_PitCom = vit_local_LocalVar%PitCom
        vit_LocalVar_PitCom_SD = vit_local_LocalVar%PitCom_SD
        vit_LocalVar_PitComAct = vit_local_LocalVar%PitComAct
        vit_LocalVar_CC_DesiredL = vit_local_LocalVar%CC_DesiredL
        vit_LocalVar_CC_ActuatedL = vit_local_LocalVar%CC_ActuatedL
        vit_LocalVar_CC_ActuatedDL = vit_local_LocalVar%CC_ActuatedDL
        vit_LocalVar_StC_Input = vit_local_LocalVar%StC_Input
        vit_LocalVar_Flp_Angle = vit_local_LocalVar%Flp_Angle
        vit_LocalVar_RootMyb_Last = vit_local_LocalVar%RootMyb_Last
        vit_LocalVar_AWC_complexangle = vit_local_LocalVar%AWC_complexangle
        vit_LocalVar_ZMQ_PitOffset = vit_local_LocalVar%ZMQ_PitOffset
        IF (C_ASSOCIATED(vit_LocalVar_WE_ptr)) vit_nested_LocalVar_WE = vit_local_LocalVar%WE
        IF (C_ASSOCIATED(vit_LocalVar_FP_ptr)) vit_nested_LocalVar_FP = vit_local_LocalVar%FP
        IF (C_ASSOCIATED(vit_LocalVar_piP_ptr)) vit_nested_LocalVar_piP = vit_local_LocalVar%piP
        IF (C_ASSOCIATED(vit_LocalVar_resP_ptr)) vit_nested_LocalVar_resP = vit_local_LocalVar%resP
        IF (C_ASSOCIATED(vit_LocalVar_rlP_ptr)) vit_nested_LocalVar_rlP = vit_local_LocalVar%rlP
        IF (ALLOCATED(vit_local_LocalVar%ACC_INFILE)) DEALLOCATE(vit_local_LocalVar%ACC_INFILE)
        IF (ALLOCATED(vit_local_ExtDLL%avrSWAP)) DEALLOCATE(vit_local_ExtDLL%avrSWAP)
        IF (ALLOCATED(vit_local_ErrVar%ErrMsg)) THEN
            IF (LEN(vit_local_ErrVar%ErrMsg) <= vit_ErrVar_ErrMsg_cap) THEN
                DO vit_dc_ErrVar_ErrMsg = 1, LEN(vit_local_ErrVar%ErrMsg)
                    vit_ErrVar_ErrMsg(vit_dc_ErrVar_ErrMsg) = &
                        vit_local_ErrVar%ErrMsg(vit_dc_ErrVar_ErrMsg:vit_dc_ErrVar_ErrMsg)
                END DO
                vit_ErrVar_ErrMsg_n = INT(LEN(vit_local_ErrVar%ErrMsg), C_INT)
            ELSE
                WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                    'VIT bridge: ErrVar%ErrMsg assignment of ', LEN(vit_local_ErrVar%ErrMsg), &
                    ' bytes exceeds the ', vit_ErrVar_ErrMsg_cap, &
                    '-byte buffer the caller supplied; left unchanged'
            END IF
        END IF
        IF (ALLOCATED(vit_local_ErrVar%ErrMsg)) DEALLOCATE(vit_local_ErrVar%ErrMsg)
    END SUBROUTINE extcontroller_f90