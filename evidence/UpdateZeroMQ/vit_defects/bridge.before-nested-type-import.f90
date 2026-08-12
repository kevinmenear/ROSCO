! VIT: Test-validate bridge for UpdateZeroMQ
! Allows C++ test harness to call the original Fortran function.
! Handles C↔Fortran type conversions for derived types and CHARACTER.
    SUBROUTINE updatezeromq_f90(vit_LocalVar_iStatus, vit_LocalVar_AlreadyInitialized, &
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
        vit_LocalVar_ACC_INFILE_n, vit_LocalVar_ACC_INFILE_cap, vit_LocalVar_restart, &
        vit_LocalVar_AWC_complexangle, vit_LocalVar_TiltMean, vit_LocalVar_YawMean, &
        vit_LocalVar_ZMQ_ID, vit_LocalVar_ZMQ_YawOffset, vit_LocalVar_ZMQ_TorqueOffset, &
        vit_LocalVar_ZMQ_PitOffset, vit_LocalVar_ZMQ_R_Speed, vit_LocalVar_ZMQ_R_Torque, &
        vit_LocalVar_ZMQ_R_Pitch, vit_LocalVar_WE_ptr, vit_LocalVar_FP_ptr, vit_LocalVar_piP_ptr, &
        vit_LocalVar_resP_ptr, vit_LocalVar_rlP_ptr, vit_CntrPar_ZMQ_ID, vit_CntrPar_LoggingLevel, &
        vit_CntrPar_Echo, vit_CntrPar_Ext_Interface, vit_CntrPar_DT_Out, vit_CntrPar_n_DT_Out, &
        vit_CntrPar_n_DT_ZMQ, vit_CntrPar_F_LPFType, vit_CntrPar_F_LPFCornerFreq, &
        vit_CntrPar_F_LPFDamping, vit_CntrPar_F_NumNotchFilts, vit_CntrPar_F_GenSpdNotch_N, &
        vit_CntrPar_F_GenSpdNotch_Ind_ptr, vit_CntrPar_F_GenSpdNotch_Ind_n, &
        vit_CntrPar_F_GenSpdNotch_Ind_cap, vit_CntrPar_F_TwrTopNotch_N, &
        vit_CntrPar_F_TwrTopNotch_Ind_ptr, vit_CntrPar_F_TwrTopNotch_Ind_n, &
        vit_CntrPar_F_TwrTopNotch_Ind_cap, vit_CntrPar_F_NotchFreqs_ptr, &
        vit_CntrPar_F_NotchFreqs_n, vit_CntrPar_F_NotchFreqs_cap, vit_CntrPar_F_NotchBetaNum_ptr, &
        vit_CntrPar_F_NotchBetaNum_n, vit_CntrPar_F_NotchBetaNum_cap, &
        vit_CntrPar_F_NotchBetaDen_ptr, vit_CntrPar_F_NotchBetaDen_n, &
        vit_CntrPar_F_NotchBetaDen_cap, vit_CntrPar_F_SSCornerFreq, vit_CntrPar_F_WECornerFreq, &
        vit_CntrPar_F_FlCornerFreq_ptr, vit_CntrPar_F_FlCornerFreq_n, &
        vit_CntrPar_F_FlCornerFreq_cap, vit_CntrPar_F_FlHighPassFreq, vit_CntrPar_F_YawErr, &
        vit_CntrPar_F_FlpCornerFreq_ptr, vit_CntrPar_F_FlpCornerFreq_n, &
        vit_CntrPar_F_FlpCornerFreq_cap, vit_CntrPar_F_VSRefSpdCornerFreq, vit_CntrPar_TRA_Mode, &
        vit_CntrPar_TRA_ExclSpeed, vit_CntrPar_TRA_ExclBand, vit_CntrPar_TRA_RateLimit, &
        vit_CntrPar_TD_Mode, vit_CntrPar_FA_HPFCornerFreq, vit_CntrPar_FA_IntSat, &
        vit_CntrPar_FA_KI, vit_CntrPar_IPC_ControlMode, vit_CntrPar_IPC_Vramp_ptr, &
        vit_CntrPar_IPC_Vramp_n, vit_CntrPar_IPC_Vramp_cap, vit_CntrPar_IPC_IntSat, &
        vit_CntrPar_IPC_SatMode, vit_CntrPar_IPC_KP_ptr, vit_CntrPar_IPC_KP_n, &
        vit_CntrPar_IPC_KP_cap, vit_CntrPar_IPC_KI_ptr, vit_CntrPar_IPC_KI_n, &
        vit_CntrPar_IPC_KI_cap, vit_CntrPar_IPC_aziOffset_ptr, vit_CntrPar_IPC_aziOffset_n, &
        vit_CntrPar_IPC_aziOffset_cap, vit_CntrPar_IPC_CornerFreqAct, vit_CntrPar_PC_ControlMode, &
        vit_CntrPar_PC_GS_n, vit_CntrPar_PC_GS_angles_ptr, vit_CntrPar_PC_GS_angles_n, &
        vit_CntrPar_PC_GS_angles_cap, vit_CntrPar_PC_GS_KP_ptr, vit_CntrPar_PC_GS_KP_n, &
        vit_CntrPar_PC_GS_KP_cap, vit_CntrPar_PC_GS_KI_ptr, vit_CntrPar_PC_GS_KI_n, &
        vit_CntrPar_PC_GS_KI_cap, vit_CntrPar_PC_GS_KD_ptr, vit_CntrPar_PC_GS_KD_n, &
        vit_CntrPar_PC_GS_KD_cap, vit_CntrPar_PC_GS_TF_ptr, vit_CntrPar_PC_GS_TF_n, &
        vit_CntrPar_PC_GS_TF_cap, vit_CntrPar_PC_MaxPit, vit_CntrPar_PC_MinPit, &
        vit_CntrPar_PC_MaxRat, vit_CntrPar_PC_MinRat, vit_CntrPar_PC_RefSpd, &
        vit_CntrPar_PC_FinePit, vit_CntrPar_PC_Switch, vit_CntrPar_VS_ControlMode, &
        vit_CntrPar_VS_ConstPower, vit_CntrPar_VS_FBP, vit_CntrPar_VS_GenEff, &
        vit_CntrPar_VS_ArSatTq, vit_CntrPar_VS_MaxRat, vit_CntrPar_VS_MaxTq, vit_CntrPar_VS_MinTq, &
        vit_CntrPar_VS_MinOMSpd, vit_CntrPar_VS_Rgn2K, vit_CntrPar_VS_RtPwr, vit_CntrPar_VS_RtTq, &
        vit_CntrPar_VS_RefSpd, vit_CntrPar_VS_n, vit_CntrPar_VS_KP_ptr, vit_CntrPar_VS_KP_n, &
        vit_CntrPar_VS_KP_cap, vit_CntrPar_VS_KI_ptr, vit_CntrPar_VS_KI_n, vit_CntrPar_VS_KI_cap, &
        vit_CntrPar_VS_TSRopt, vit_CntrPar_VS_FBP_n, vit_CntrPar_VS_FBP_U_ptr, &
        vit_CntrPar_VS_FBP_U_n, vit_CntrPar_VS_FBP_U_cap, vit_CntrPar_VS_FBP_Omega_ptr, &
        vit_CntrPar_VS_FBP_Omega_n, vit_CntrPar_VS_FBP_Omega_cap, vit_CntrPar_VS_FBP_Tau_ptr, &
        vit_CntrPar_VS_FBP_Tau_n, vit_CntrPar_VS_FBP_Tau_cap, vit_CntrPar_SS_Mode, &
        vit_CntrPar_SS_VSGain, vit_CntrPar_SS_PCGain, vit_CntrPar_PRC_Mode, vit_CntrPar_PRC_Comm, &
        vit_CntrPar_PRC_WindSpeeds_ptr, vit_CntrPar_PRC_WindSpeeds_n, &
        vit_CntrPar_PRC_WindSpeeds_cap, vit_CntrPar_PRC_GenSpeeds_ptr, vit_CntrPar_PRC_GenSpeeds_n, &
        vit_CntrPar_PRC_GenSpeeds_cap, vit_CntrPar_PRC_n, vit_CntrPar_PRC_LPF_Freq, &
        vit_CntrPar_PRC_R_Torque, vit_CntrPar_PRC_R_Speed, vit_CntrPar_PRC_R_Pitch, &
        vit_CntrPar_PRC_Table_n, vit_CntrPar_PRC_Pitch_Table_ptr, vit_CntrPar_PRC_Pitch_Table_n, &
        vit_CntrPar_PRC_Pitch_Table_cap, vit_CntrPar_PRC_R_Table_ptr, vit_CntrPar_PRC_R_Table_n, &
        vit_CntrPar_PRC_R_Table_cap, vit_CntrPar_WE_Mode, vit_CntrPar_WE_BladeRadius, &
        vit_CntrPar_WE_CP_n, vit_CntrPar_WE_CP_ptr, vit_CntrPar_WE_CP_n_v2, vit_CntrPar_WE_CP_cap, &
        vit_CntrPar_WE_Gamma, vit_CntrPar_WE_GearboxRatio, vit_CntrPar_WE_Jtot, &
        vit_CntrPar_WE_RhoAir, vit_CntrPar_PerfFileName, vit_CntrPar_PerfTableSize_ptr, &
        vit_CntrPar_PerfTableSize_n, vit_CntrPar_PerfTableSize_cap, vit_CntrPar_WE_FOPoles_N, &
        vit_CntrPar_WE_FOPoles_v_ptr, vit_CntrPar_WE_FOPoles_v_n, vit_CntrPar_WE_FOPoles_v_cap, &
        vit_CntrPar_WE_FOPoles_ptr, vit_CntrPar_WE_FOPoles_n_v2, vit_CntrPar_WE_FOPoles_cap, &
        vit_CntrPar_Y_ControlMode, vit_CntrPar_Y_uSwitch, vit_CntrPar_Y_ErrThresh_ptr, &
        vit_CntrPar_Y_ErrThresh_n, vit_CntrPar_Y_ErrThresh_cap, vit_CntrPar_Y_Rate, &
        vit_CntrPar_Y_MErrSet, vit_CntrPar_Y_IPC_IntSat, vit_CntrPar_Y_IPC_KP, &
        vit_CntrPar_Y_IPC_KI, vit_CntrPar_PS_Mode, vit_CntrPar_PS_BldPitchMin_N, &
        vit_CntrPar_PS_WindSpeeds_ptr, vit_CntrPar_PS_WindSpeeds_n, vit_CntrPar_PS_WindSpeeds_cap, &
        vit_CntrPar_PS_BldPitchMin_ptr, vit_CntrPar_PS_BldPitchMin_n_v2, &
        vit_CntrPar_PS_BldPitchMin_cap, vit_CntrPar_SU_Mode, vit_CntrPar_SU_StartTime, &
        vit_CntrPar_SU_FW_MinDuration, vit_CntrPar_SU_RotorSpeedThresh, &
        vit_CntrPar_SU_RotorSpeedCornerFreq, vit_CntrPar_SU_LoadStages_N, &
        vit_CntrPar_SU_LoadStages_ptr, vit_CntrPar_SU_LoadStages_n_v2, &
        vit_CntrPar_SU_LoadStages_cap, vit_CntrPar_SU_LoadRampDuration_ptr, &
        vit_CntrPar_SU_LoadRampDuration_n, vit_CntrPar_SU_LoadRampDuration_cap, &
        vit_CntrPar_SU_LoadHoldDuration_ptr, vit_CntrPar_SU_LoadHoldDuration_n, &
        vit_CntrPar_SU_LoadHoldDuration_cap, vit_CntrPar_SD_Mode, vit_CntrPar_SD_TimeActivate, &
        vit_CntrPar_SD_EnablePitch, vit_CntrPar_SD_EnableYawError, vit_CntrPar_SD_EnableGenSpeed, &
        vit_CntrPar_SD_EnableTime, vit_CntrPar_SD_MaxPit, vit_CntrPar_SD_PitchCornerFreq, &
        vit_CntrPar_SD_MaxYawError, vit_CntrPar_SD_YawErrorCornerFreq, vit_CntrPar_SD_MaxGenSpd, &
        vit_CntrPar_SD_GenSpdCornerFreq, vit_CntrPar_SD_Time, vit_CntrPar_SD_Method, &
        vit_CntrPar_SD_MaxTorqueRate_ptr, vit_CntrPar_SD_MaxTorqueRate_n, &
        vit_CntrPar_SD_MaxTorqueRate_cap, vit_CntrPar_SD_MaxPitchRate_ptr, &
        vit_CntrPar_SD_MaxPitchRate_n, vit_CntrPar_SD_MaxPitchRate_cap, &
        vit_CntrPar_SD_StagePitch_ptr, vit_CntrPar_SD_StagePitch_n, vit_CntrPar_SD_StagePitch_cap, &
        vit_CntrPar_SD_StageTime_ptr, vit_CntrPar_SD_StageTime_n, vit_CntrPar_SD_StageTime_cap, &
        vit_CntrPar_SD_Stage_N, vit_CntrPar_Fl_Mode, vit_CntrPar_Fl_n, vit_CntrPar_Fl_Kp_ptr, &
        vit_CntrPar_Fl_Kp_n, vit_CntrPar_Fl_Kp_cap, vit_CntrPar_Fl_U_ptr, vit_CntrPar_Fl_U_n, &
        vit_CntrPar_Fl_U_cap, vit_CntrPar_Flp_Mode, vit_CntrPar_Flp_Angle, vit_CntrPar_Flp_Kp, &
        vit_CntrPar_Flp_Ki, vit_CntrPar_Flp_MaxPit, vit_CntrPar_OL_Filename, vit_CntrPar_OL_Mode, &
        vit_CntrPar_OL_BP_Mode, vit_CntrPar_OL_BP_FiltFreq, vit_CntrPar_Ind_Breakpoint, &
        vit_CntrPar_Ind_BldPitch_ptr, vit_CntrPar_Ind_BldPitch_n, vit_CntrPar_Ind_BldPitch_cap, &
        vit_CntrPar_Ind_GenTq, vit_CntrPar_Ind_YawRate, vit_CntrPar_Ind_R_Speed, &
        vit_CntrPar_Ind_R_Torque, vit_CntrPar_Ind_R_Pitch, vit_CntrPar_Ind_Azimuth, &
        vit_CntrPar_RP_Gains_ptr, vit_CntrPar_RP_Gains_n, vit_CntrPar_RP_Gains_cap, &
        vit_CntrPar_Ind_CableControl_ptr, vit_CntrPar_Ind_CableControl_n, &
        vit_CntrPar_Ind_CableControl_cap, vit_CntrPar_Ind_StructControl_ptr, &
        vit_CntrPar_Ind_StructControl_n, vit_CntrPar_Ind_StructControl_cap, &
        vit_CntrPar_OL_Breakpoints_ptr, vit_CntrPar_OL_Breakpoints_n, &
        vit_CntrPar_OL_Breakpoints_cap, vit_CntrPar_OL_BldPitch1_ptr, vit_CntrPar_OL_BldPitch1_n, &
        vit_CntrPar_OL_BldPitch1_cap, vit_CntrPar_OL_BldPitch2_ptr, vit_CntrPar_OL_BldPitch2_n, &
        vit_CntrPar_OL_BldPitch2_cap, vit_CntrPar_OL_BldPitch3_ptr, vit_CntrPar_OL_BldPitch3_n, &
        vit_CntrPar_OL_BldPitch3_cap, vit_CntrPar_OL_CableControl_ptr, &
        vit_CntrPar_OL_CableControl_rows, vit_CntrPar_OL_CableControl_cols, &
        vit_CntrPar_OL_StructControl_ptr, vit_CntrPar_OL_StructControl_rows, &
        vit_CntrPar_OL_StructControl_cols, vit_CntrPar_OL_GenTq_ptr, vit_CntrPar_OL_GenTq_n, &
        vit_CntrPar_OL_GenTq_cap, vit_CntrPar_OL_YawRate_ptr, vit_CntrPar_OL_YawRate_n, &
        vit_CntrPar_OL_YawRate_cap, vit_CntrPar_OL_Azimuth_ptr, vit_CntrPar_OL_Azimuth_n, &
        vit_CntrPar_OL_Azimuth_cap, vit_CntrPar_OL_R_Speed_ptr, vit_CntrPar_OL_R_Speed_n, &
        vit_CntrPar_OL_R_Speed_cap, vit_CntrPar_OL_R_Torque_ptr, vit_CntrPar_OL_R_Torque_n, &
        vit_CntrPar_OL_R_Torque_cap, vit_CntrPar_OL_R_Pitch_ptr, vit_CntrPar_OL_R_Pitch_n, &
        vit_CntrPar_OL_R_Pitch_cap, vit_CntrPar_OL_Channels_ptr, vit_CntrPar_OL_Channels_rows, &
        vit_CntrPar_OL_Channels_cols, vit_CntrPar_PA_Mode, vit_CntrPar_PA_CornerFreq, &
        vit_CntrPar_PA_Damping, vit_CntrPar_AWC_Mode, vit_CntrPar_AWC_NumModes, &
        vit_CntrPar_AWC_n_ptr, vit_CntrPar_AWC_n_n, vit_CntrPar_AWC_n_cap, &
        vit_CntrPar_AWC_harmonic_ptr, vit_CntrPar_AWC_harmonic_n, vit_CntrPar_AWC_harmonic_cap, &
        vit_CntrPar_AWC_freq_ptr, vit_CntrPar_AWC_freq_n, vit_CntrPar_AWC_freq_cap, &
        vit_CntrPar_AWC_amp_ptr, vit_CntrPar_AWC_amp_n, vit_CntrPar_AWC_amp_cap, &
        vit_CntrPar_AWC_clockangle_ptr, vit_CntrPar_AWC_clockangle_n, &
        vit_CntrPar_AWC_clockangle_cap, vit_CntrPar_AWC_phaseoffset, vit_CntrPar_AWC_CntrGains_ptr, &
        vit_CntrPar_AWC_CntrGains_n, vit_CntrPar_AWC_CntrGains_cap, vit_CntrPar_PF_Mode, &
        vit_CntrPar_PF_Offsets_ptr, vit_CntrPar_PF_Offsets_n, vit_CntrPar_PF_Offsets_cap, &
        vit_CntrPar_PF_TimeStuck_ptr, vit_CntrPar_PF_TimeStuck_n, vit_CntrPar_PF_TimeStuck_cap, &
        vit_CntrPar_Ext_Mode, vit_CntrPar_DLL_FileName, vit_CntrPar_DLL_InFile, &
        vit_CntrPar_DLL_ProcName, vit_CntrPar_ZMQ_Mode, vit_CntrPar_ZMQ_CommAddress, &
        vit_CntrPar_ZMQ_UpdatePeriod, vit_CntrPar_CC_Mode, vit_CntrPar_CC_Group_N, &
        vit_CntrPar_CC_ActTau, vit_CntrPar_CC_GroupIndex_ptr, vit_CntrPar_CC_GroupIndex_n, &
        vit_CntrPar_CC_GroupIndex_cap, vit_CntrPar_StC_Mode, vit_CntrPar_StC_Group_N, &
        vit_CntrPar_StC_GroupIndex_ptr, vit_CntrPar_StC_GroupIndex_n, &
        vit_CntrPar_StC_GroupIndex_cap, vit_CntrPar_PC_RtTq99, vit_CntrPar_VS_MaxOMTq, &
        vit_CntrPar_VS_MinOMTq, vit_ErrVar_size_avcMSG, vit_ErrVar_aviFAIL, vit_ErrVar_ErrStat, &
        vit_ErrVar_ErrMsg, vit_ErrVar_ErrMsg_n, vit_ErrVar_ErrMsg_cap) &
        BIND(C, NAME='updatezeromq_f90')
        USE ISO_C_BINDING
        USE, INTRINSIC :: ISO_FORTRAN_ENV, ONLY: ERROR_UNIT
        USE ZeroMQInterface
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
        IMPLICIT NONE
        INTEGER(C_INT) :: vit_LocalVar_iStatus
        INTEGER(C_INT) :: vit_LocalVar_AlreadyInitialized
        INTEGER(C_INT) :: vit_LocalVar_RestartWSE
        REAL(C_DOUBLE) :: vit_LocalVar_Time
        REAL(C_DOUBLE) :: vit_LocalVar_DT
        LOGICAL(C_BOOL) :: vit_LocalVar_WriteThisStep
        INTEGER(C_INT) :: vit_LocalVar_n_DT
        REAL(C_DOUBLE) :: vit_LocalVar_Time_Last
        REAL(C_DOUBLE) :: vit_LocalVar_VS_GenPwr
        REAL(C_DOUBLE) :: vit_LocalVar_GenSpeed
        REAL(C_DOUBLE) :: vit_LocalVar_RotSpeed
        REAL(C_DOUBLE) :: vit_LocalVar_NacHeading
        REAL(C_DOUBLE) :: vit_LocalVar_NacVane
        REAL(C_DOUBLE) :: vit_LocalVar_NacVaneF
        REAL(C_DOUBLE) :: vit_LocalVar_WindDir
        REAL(C_DOUBLE) :: vit_LocalVar_HorWindV
        REAL(C_DOUBLE) :: vit_LocalVar_HorWindV_F
        REAL(C_DOUBLE) :: vit_LocalVar_rootMOOP(3)
        REAL(C_DOUBLE) :: vit_LocalVar_rootMOOPF(3)
        REAL(C_DOUBLE) :: vit_LocalVar_BlPitch(3)
        REAL(C_DOUBLE) :: vit_LocalVar_BlPitchCMeas
        REAL(C_DOUBLE) :: vit_LocalVar_Azimuth
        REAL(C_DOUBLE) :: vit_LocalVar_OL_Azimuth
        REAL(C_DOUBLE) :: vit_LocalVar_AzUnwrapped
        REAL(C_DOUBLE) :: vit_LocalVar_AzError
        REAL(C_DOUBLE) :: vit_LocalVar_GenTqAz
        REAL(C_DOUBLE) :: vit_LocalVar_AzBuffer(2)
        INTEGER(C_INT) :: vit_LocalVar_NumBl
        REAL(C_DOUBLE) :: vit_LocalVar_FA_Acc_TT
        REAL(C_DOUBLE) :: vit_LocalVar_SS_Acc_TT
        REAL(C_DOUBLE) :: vit_LocalVar_FA_Acc_Nac
        REAL(C_DOUBLE) :: vit_LocalVar_NacIMU_FA_RAcc
        REAL(C_DOUBLE) :: vit_LocalVar_FA_AccHPF
        REAL(C_DOUBLE) :: vit_LocalVar_FA_AccHPFI
        REAL(C_DOUBLE) :: vit_LocalVar_FA_PitCom(3)
        REAL(C_DOUBLE) :: vit_LocalVar_VS_RefSpd
        REAL(C_DOUBLE) :: vit_LocalVar_VS_RefSpd_TSR
        REAL(C_DOUBLE) :: vit_LocalVar_VS_RefSpd_TRA
        REAL(C_DOUBLE) :: vit_LocalVar_VS_RefSpd_RL
        REAL(C_DOUBLE) :: vit_LocalVar_PC_RefSpd
        REAL(C_DOUBLE) :: vit_LocalVar_PC_RefSpd_SS
        REAL(C_DOUBLE) :: vit_LocalVar_PC_RefSpd_PRC
        REAL(C_DOUBLE) :: vit_LocalVar_RotSpeedF
        REAL(C_DOUBLE) :: vit_LocalVar_GenSpeedF
        REAL(C_DOUBLE) :: vit_LocalVar_GenTq
        REAL(C_DOUBLE) :: vit_LocalVar_GenTqMeas
        REAL(C_DOUBLE) :: vit_LocalVar_GenArTq
        REAL(C_DOUBLE) :: vit_LocalVar_GenBrTq
        REAL(C_DOUBLE) :: vit_LocalVar_VS_KOmega2_GenTq
        REAL(C_DOUBLE) :: vit_LocalVar_VS_ConstPwr_GenTq
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_PitComF(3)
        REAL(C_DOUBLE) :: vit_LocalVar_PC_KP
        REAL(C_DOUBLE) :: vit_LocalVar_PC_KI
        REAL(C_DOUBLE) :: vit_LocalVar_PC_KD
        REAL(C_DOUBLE) :: vit_LocalVar_PC_TF
        REAL(C_DOUBLE) :: vit_LocalVar_PC_MaxPit
        REAL(C_DOUBLE) :: vit_LocalVar_PC_MinPit
        REAL(C_DOUBLE) :: vit_LocalVar_PC_PitComT
        REAL(C_DOUBLE) :: vit_LocalVar_PC_PitComT_Last
        REAL(C_DOUBLE) :: vit_LocalVar_BlPitchCMeasF
        REAL(C_DOUBLE) :: vit_LocalVar_PC_PitComT_IPC(3)
        REAL(C_DOUBLE) :: vit_LocalVar_PC_PwrErr
        REAL(C_DOUBLE) :: vit_LocalVar_PC_SpdErr
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_AxisTilt_1P
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_AxisYaw_1P
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_AxisTilt_2P
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_AxisYaw_2P
        REAL(C_DOUBLE) :: vit_LocalVar_axisTilt_1P
        REAL(C_DOUBLE) :: vit_LocalVar_axisYaw_1P
        REAL(C_DOUBLE) :: vit_LocalVar_axisYawF_1P
        REAL(C_DOUBLE) :: vit_LocalVar_axisTilt_2P
        REAL(C_DOUBLE) :: vit_LocalVar_axisYaw_2P
        REAL(C_DOUBLE) :: vit_LocalVar_axisYawF_2P
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_KI(2)
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_KP(2)
        REAL(C_DOUBLE) :: vit_LocalVar_IPC_IntSat
        INTEGER(C_INT) :: vit_LocalVar_PC_State
        REAL(C_DOUBLE) :: vit_LocalVar_PitCom(3)
        REAL(C_DOUBLE) :: vit_LocalVar_PitCom_SD(3)
        REAL(C_DOUBLE) :: vit_LocalVar_PitComAct(3)
        REAL(C_DOUBLE) :: vit_LocalVar_SS_DelOmegaF
        REAL(C_DOUBLE) :: vit_LocalVar_TestType
        REAL(C_DOUBLE) :: vit_LocalVar_Kp_Float
        REAL(C_DOUBLE) :: vit_LocalVar_VS_MaxTq
        REAL(C_DOUBLE) :: vit_LocalVar_VS_LastGenTrq
        REAL(C_DOUBLE) :: vit_LocalVar_VS_LastGenPwr
        REAL(C_DOUBLE) :: vit_LocalVar_VS_MechGenPwr
        REAL(C_DOUBLE) :: vit_LocalVar_VS_SpdErrAr
        REAL(C_DOUBLE) :: vit_LocalVar_VS_SpdErrBr
        REAL(C_DOUBLE) :: vit_LocalVar_VS_SpdErr
        INTEGER(C_INT) :: vit_LocalVar_VS_State
        REAL(C_DOUBLE) :: vit_LocalVar_VS_Rgn3Pitch
        REAL(C_DOUBLE) :: vit_LocalVar_WE_Vw
        REAL(C_DOUBLE) :: vit_LocalVar_WE_Vw_F
        REAL(C_DOUBLE) :: vit_LocalVar_WE_VwI
        REAL(C_DOUBLE) :: vit_LocalVar_WE_VwIdot
        INTEGER(C_INT) :: vit_LocalVar_WE_Op
        INTEGER(C_INT) :: vit_LocalVar_WE_Op_Last
        REAL(C_DOUBLE) :: vit_LocalVar_VS_LastGenTrqF
        REAL(C_DOUBLE) :: vit_LocalVar_PRC_WSE_F
        REAL(C_DOUBLE) :: vit_LocalVar_PRC_R_Speed
        REAL(C_DOUBLE) :: vit_LocalVar_PRC_R_Torque
        REAL(C_DOUBLE) :: vit_LocalVar_PRC_R_Pitch
        REAL(C_DOUBLE) :: vit_LocalVar_PRC_R_Total
        REAL(C_DOUBLE) :: vit_LocalVar_PRC_Min_Pitch
        REAL(C_DOUBLE) :: vit_LocalVar_PS_Min_Pitch
        REAL(C_DOUBLE) :: vit_LocalVar_OL_Index
        INTEGER(C_INT) :: vit_LocalVar_SU_Stage
        REAL(C_DOUBLE) :: vit_LocalVar_SU_LoadStageStartTime
        REAL(C_DOUBLE) :: vit_LocalVar_SU_RotSpeedF
        INTEGER(C_INT) :: vit_LocalVar_SD_Trigger
        REAL(C_DOUBLE) :: vit_LocalVar_SD_BlPitchF
        REAL(C_DOUBLE) :: vit_LocalVar_SD_NacVaneF
        REAL(C_DOUBLE) :: vit_LocalVar_SD_GenSpeedF
        INTEGER(C_INT) :: vit_LocalVar_SD_Stage
        REAL(C_DOUBLE) :: vit_LocalVar_SD_StageStartTime
        REAL(C_DOUBLE) :: vit_LocalVar_SD_MaxPitchRate
        REAL(C_DOUBLE) :: vit_LocalVar_SD_MaxTorqueRate
        REAL(C_DOUBLE) :: vit_LocalVar_GenTq_SD
        REAL(C_DOUBLE) :: vit_LocalVar_Fl_PitCom
        REAL(C_DOUBLE) :: vit_LocalVar_NACIMU_FA_AccF
        REAL(C_DOUBLE) :: vit_LocalVar_FA_AccF
        INTEGER(C_INT) :: vit_LocalVar_FA_Hist
        REAL(C_DOUBLE) :: vit_LocalVar_TRA_LastRefSpd
        REAL(C_DOUBLE) :: vit_LocalVar_VS_RefSpeed
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTDX
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTDY
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTDZ
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRDX
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRDY
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRDZ
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTVX
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTVY
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTVZ
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRVX
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRVY
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRVZ
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTAX
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTAY
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmTAZ
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRAX
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRAY
        REAL(C_DOUBLE) :: vit_LocalVar_PtfmRAZ
        REAL(C_DOUBLE) :: vit_LocalVar_CC_DesiredL(12)
        REAL(C_DOUBLE) :: vit_LocalVar_CC_ActuatedL(12)
        REAL(C_DOUBLE) :: vit_LocalVar_CC_ActuatedDL(12)
        REAL(C_DOUBLE) :: vit_LocalVar_StC_Input(12)
        REAL(C_DOUBLE) :: vit_LocalVar_Flp_Angle(3)
        REAL(C_DOUBLE) :: vit_LocalVar_RootMyb_Last(3)
        INTEGER(C_INT) :: vit_LocalVar_ACC_INFILE_SIZE
        TYPE(C_PTR), VALUE :: vit_LocalVar_ACC_INFILE_ptr
        INTEGER(C_INT) :: vit_LocalVar_ACC_INFILE_n
        INTEGER(C_INT), VALUE :: vit_LocalVar_ACC_INFILE_cap
        LOGICAL(C_BOOL) :: vit_LocalVar_restart
        COMPLEX(C_DOUBLE_COMPLEX) :: vit_LocalVar_AWC_complexangle(3)
        REAL(C_DOUBLE) :: vit_LocalVar_TiltMean
        REAL(C_DOUBLE) :: vit_LocalVar_YawMean
        INTEGER(C_INT) :: vit_LocalVar_ZMQ_ID
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_YawOffset
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_TorqueOffset
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_PitOffset(3)
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_R_Speed
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_R_Torque
        REAL(C_DOUBLE) :: vit_LocalVar_ZMQ_R_Pitch
        TYPE(C_PTR), VALUE :: vit_LocalVar_WE_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_FP_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_piP_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_resP_ptr
        TYPE(C_PTR), VALUE :: vit_LocalVar_rlP_ptr
        INTEGER(C_INT) :: vit_CntrPar_ZMQ_ID
        INTEGER(C_INT) :: vit_CntrPar_LoggingLevel
        INTEGER(C_INT) :: vit_CntrPar_Echo
        INTEGER(C_INT) :: vit_CntrPar_Ext_Interface
        REAL(C_DOUBLE) :: vit_CntrPar_DT_Out
        INTEGER(C_INT) :: vit_CntrPar_n_DT_Out
        INTEGER(C_INT) :: vit_CntrPar_n_DT_ZMQ
        INTEGER(C_INT) :: vit_CntrPar_F_LPFType
        REAL(C_DOUBLE) :: vit_CntrPar_F_LPFCornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_F_LPFDamping
        INTEGER(C_INT) :: vit_CntrPar_F_NumNotchFilts
        INTEGER(C_INT) :: vit_CntrPar_F_GenSpdNotch_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_GenSpdNotch_Ind_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_GenSpdNotch_Ind_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_GenSpdNotch_Ind_cap
        INTEGER(C_INT) :: vit_CntrPar_F_TwrTopNotch_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_TwrTopNotch_Ind_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_TwrTopNotch_Ind_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_TwrTopNotch_Ind_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_NotchFreqs_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_NotchFreqs_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NotchFreqs_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_NotchBetaNum_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_NotchBetaNum_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NotchBetaNum_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_NotchBetaDen_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_NotchBetaDen_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_NotchBetaDen_cap
        REAL(C_DOUBLE) :: vit_CntrPar_F_SSCornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_F_WECornerFreq
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_FlCornerFreq_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_FlCornerFreq_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_FlCornerFreq_cap
        REAL(C_DOUBLE) :: vit_CntrPar_F_FlHighPassFreq
        REAL(C_DOUBLE) :: vit_CntrPar_F_YawErr
        TYPE(C_PTR), VALUE :: vit_CntrPar_F_FlpCornerFreq_ptr
        INTEGER(C_INT) :: vit_CntrPar_F_FlpCornerFreq_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_F_FlpCornerFreq_cap
        REAL(C_DOUBLE) :: vit_CntrPar_F_VSRefSpdCornerFreq
        INTEGER(C_INT) :: vit_CntrPar_TRA_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_TRA_ExclSpeed
        REAL(C_DOUBLE) :: vit_CntrPar_TRA_ExclBand
        REAL(C_DOUBLE) :: vit_CntrPar_TRA_RateLimit
        INTEGER(C_INT) :: vit_CntrPar_TD_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_FA_HPFCornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_FA_IntSat
        REAL(C_DOUBLE) :: vit_CntrPar_FA_KI
        INTEGER(C_INT) :: vit_CntrPar_IPC_ControlMode
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_Vramp_ptr
        INTEGER(C_INT) :: vit_CntrPar_IPC_Vramp_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_Vramp_cap
        REAL(C_DOUBLE) :: vit_CntrPar_IPC_IntSat
        INTEGER(C_INT) :: vit_CntrPar_IPC_SatMode
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_KP_ptr
        INTEGER(C_INT) :: vit_CntrPar_IPC_KP_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_KP_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_KI_ptr
        INTEGER(C_INT) :: vit_CntrPar_IPC_KI_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_KI_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_IPC_aziOffset_ptr
        INTEGER(C_INT) :: vit_CntrPar_IPC_aziOffset_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_IPC_aziOffset_cap
        REAL(C_DOUBLE) :: vit_CntrPar_IPC_CornerFreqAct
        INTEGER(C_INT) :: vit_CntrPar_PC_ControlMode
        INTEGER(C_INT) :: vit_CntrPar_PC_GS_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_angles_ptr
        INTEGER(C_INT) :: vit_CntrPar_PC_GS_angles_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_angles_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_KP_ptr
        INTEGER(C_INT) :: vit_CntrPar_PC_GS_KP_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_KP_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_KI_ptr
        INTEGER(C_INT) :: vit_CntrPar_PC_GS_KI_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_KI_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_KD_ptr
        INTEGER(C_INT) :: vit_CntrPar_PC_GS_KD_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_KD_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PC_GS_TF_ptr
        INTEGER(C_INT) :: vit_CntrPar_PC_GS_TF_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PC_GS_TF_cap
        REAL(C_DOUBLE) :: vit_CntrPar_PC_MaxPit
        REAL(C_DOUBLE) :: vit_CntrPar_PC_MinPit
        REAL(C_DOUBLE) :: vit_CntrPar_PC_MaxRat
        REAL(C_DOUBLE) :: vit_CntrPar_PC_MinRat
        REAL(C_DOUBLE) :: vit_CntrPar_PC_RefSpd
        REAL(C_DOUBLE) :: vit_CntrPar_PC_FinePit
        REAL(C_DOUBLE) :: vit_CntrPar_PC_Switch
        INTEGER(C_INT) :: vit_CntrPar_VS_ControlMode
        INTEGER(C_INT) :: vit_CntrPar_VS_ConstPower
        INTEGER(C_INT) :: vit_CntrPar_VS_FBP
        REAL(C_DOUBLE) :: vit_CntrPar_VS_GenEff
        REAL(C_DOUBLE) :: vit_CntrPar_VS_ArSatTq
        REAL(C_DOUBLE) :: vit_CntrPar_VS_MaxRat
        REAL(C_DOUBLE) :: vit_CntrPar_VS_MaxTq
        REAL(C_DOUBLE) :: vit_CntrPar_VS_MinTq
        REAL(C_DOUBLE) :: vit_CntrPar_VS_MinOMSpd
        REAL(C_DOUBLE) :: vit_CntrPar_VS_Rgn2K
        REAL(C_DOUBLE) :: vit_CntrPar_VS_RtPwr
        REAL(C_DOUBLE) :: vit_CntrPar_VS_RtTq
        REAL(C_DOUBLE) :: vit_CntrPar_VS_RefSpd
        INTEGER(C_INT) :: vit_CntrPar_VS_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_KP_ptr
        INTEGER(C_INT) :: vit_CntrPar_VS_KP_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_KP_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_KI_ptr
        INTEGER(C_INT) :: vit_CntrPar_VS_KI_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_KI_cap
        REAL(C_DOUBLE) :: vit_CntrPar_VS_TSRopt
        INTEGER(C_INT) :: vit_CntrPar_VS_FBP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_FBP_U_ptr
        INTEGER(C_INT) :: vit_CntrPar_VS_FBP_U_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_U_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_FBP_Omega_ptr
        INTEGER(C_INT) :: vit_CntrPar_VS_FBP_Omega_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_Omega_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_VS_FBP_Tau_ptr
        INTEGER(C_INT) :: vit_CntrPar_VS_FBP_Tau_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_VS_FBP_Tau_cap
        INTEGER(C_INT) :: vit_CntrPar_SS_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_SS_VSGain
        REAL(C_DOUBLE) :: vit_CntrPar_SS_PCGain
        INTEGER(C_INT) :: vit_CntrPar_PRC_Mode
        INTEGER(C_INT) :: vit_CntrPar_PRC_Comm
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_WindSpeeds_ptr
        INTEGER(C_INT) :: vit_CntrPar_PRC_WindSpeeds_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_WindSpeeds_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_GenSpeeds_ptr
        INTEGER(C_INT) :: vit_CntrPar_PRC_GenSpeeds_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_GenSpeeds_cap
        INTEGER(C_INT) :: vit_CntrPar_PRC_n
        REAL(C_DOUBLE) :: vit_CntrPar_PRC_LPF_Freq
        REAL(C_DOUBLE) :: vit_CntrPar_PRC_R_Torque
        REAL(C_DOUBLE) :: vit_CntrPar_PRC_R_Speed
        REAL(C_DOUBLE) :: vit_CntrPar_PRC_R_Pitch
        INTEGER(C_INT) :: vit_CntrPar_PRC_Table_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_Pitch_Table_ptr
        INTEGER(C_INT) :: vit_CntrPar_PRC_Pitch_Table_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_Pitch_Table_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PRC_R_Table_ptr
        INTEGER(C_INT) :: vit_CntrPar_PRC_R_Table_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PRC_R_Table_cap
        INTEGER(C_INT) :: vit_CntrPar_WE_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_WE_BladeRadius
        INTEGER(C_INT) :: vit_CntrPar_WE_CP_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_WE_CP_ptr
        INTEGER(C_INT) :: vit_CntrPar_WE_CP_n_v2
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_CP_cap
        REAL(C_DOUBLE) :: vit_CntrPar_WE_Gamma
        REAL(C_DOUBLE) :: vit_CntrPar_WE_GearboxRatio
        REAL(C_DOUBLE) :: vit_CntrPar_WE_Jtot
        REAL(C_DOUBLE) :: vit_CntrPar_WE_RhoAir
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_PerfFileName(1024)
        TYPE(C_PTR), VALUE :: vit_CntrPar_PerfTableSize_ptr
        INTEGER(C_INT) :: vit_CntrPar_PerfTableSize_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PerfTableSize_cap
        INTEGER(C_INT) :: vit_CntrPar_WE_FOPoles_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_WE_FOPoles_v_ptr
        INTEGER(C_INT) :: vit_CntrPar_WE_FOPoles_v_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_FOPoles_v_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_WE_FOPoles_ptr
        INTEGER(C_INT) :: vit_CntrPar_WE_FOPoles_n_v2
        INTEGER(C_INT), VALUE :: vit_CntrPar_WE_FOPoles_cap
        INTEGER(C_INT) :: vit_CntrPar_Y_ControlMode
        REAL(C_DOUBLE) :: vit_CntrPar_Y_uSwitch
        TYPE(C_PTR), VALUE :: vit_CntrPar_Y_ErrThresh_ptr
        INTEGER(C_INT) :: vit_CntrPar_Y_ErrThresh_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Y_ErrThresh_cap
        REAL(C_DOUBLE) :: vit_CntrPar_Y_Rate
        REAL(C_DOUBLE) :: vit_CntrPar_Y_MErrSet
        REAL(C_DOUBLE) :: vit_CntrPar_Y_IPC_IntSat
        REAL(C_DOUBLE) :: vit_CntrPar_Y_IPC_KP
        REAL(C_DOUBLE) :: vit_CntrPar_Y_IPC_KI
        INTEGER(C_INT) :: vit_CntrPar_PS_Mode
        INTEGER(C_INT) :: vit_CntrPar_PS_BldPitchMin_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_PS_WindSpeeds_ptr
        INTEGER(C_INT) :: vit_CntrPar_PS_WindSpeeds_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PS_WindSpeeds_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PS_BldPitchMin_ptr
        INTEGER(C_INT) :: vit_CntrPar_PS_BldPitchMin_n_v2
        INTEGER(C_INT), VALUE :: vit_CntrPar_PS_BldPitchMin_cap
        INTEGER(C_INT) :: vit_CntrPar_SU_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_SU_StartTime
        REAL(C_DOUBLE) :: vit_CntrPar_SU_FW_MinDuration
        REAL(C_DOUBLE) :: vit_CntrPar_SU_RotorSpeedThresh
        REAL(C_DOUBLE) :: vit_CntrPar_SU_RotorSpeedCornerFreq
        INTEGER(C_INT) :: vit_CntrPar_SU_LoadStages_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_SU_LoadStages_ptr
        INTEGER(C_INT) :: vit_CntrPar_SU_LoadStages_n_v2
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadStages_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_SU_LoadRampDuration_ptr
        INTEGER(C_INT) :: vit_CntrPar_SU_LoadRampDuration_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadRampDuration_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_SU_LoadHoldDuration_ptr
        INTEGER(C_INT) :: vit_CntrPar_SU_LoadHoldDuration_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SU_LoadHoldDuration_cap
        INTEGER(C_INT) :: vit_CntrPar_SD_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_SD_TimeActivate
        INTEGER(C_INT) :: vit_CntrPar_SD_EnablePitch
        INTEGER(C_INT) :: vit_CntrPar_SD_EnableYawError
        INTEGER(C_INT) :: vit_CntrPar_SD_EnableGenSpeed
        INTEGER(C_INT) :: vit_CntrPar_SD_EnableTime
        REAL(C_DOUBLE) :: vit_CntrPar_SD_MaxPit
        REAL(C_DOUBLE) :: vit_CntrPar_SD_PitchCornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_SD_MaxYawError
        REAL(C_DOUBLE) :: vit_CntrPar_SD_YawErrorCornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_SD_MaxGenSpd
        REAL(C_DOUBLE) :: vit_CntrPar_SD_GenSpdCornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_SD_Time
        INTEGER(C_INT) :: vit_CntrPar_SD_Method
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_MaxTorqueRate_ptr
        INTEGER(C_INT) :: vit_CntrPar_SD_MaxTorqueRate_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_MaxTorqueRate_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_MaxPitchRate_ptr
        INTEGER(C_INT) :: vit_CntrPar_SD_MaxPitchRate_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_MaxPitchRate_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_StagePitch_ptr
        INTEGER(C_INT) :: vit_CntrPar_SD_StagePitch_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_StagePitch_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_SD_StageTime_ptr
        INTEGER(C_INT) :: vit_CntrPar_SD_StageTime_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_SD_StageTime_cap
        INTEGER(C_INT) :: vit_CntrPar_SD_Stage_N
        INTEGER(C_INT) :: vit_CntrPar_Fl_Mode
        INTEGER(C_INT) :: vit_CntrPar_Fl_n
        TYPE(C_PTR), VALUE :: vit_CntrPar_Fl_Kp_ptr
        INTEGER(C_INT) :: vit_CntrPar_Fl_Kp_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Fl_Kp_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_Fl_U_ptr
        INTEGER(C_INT) :: vit_CntrPar_Fl_U_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Fl_U_cap
        INTEGER(C_INT) :: vit_CntrPar_Flp_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_Flp_Angle
        REAL(C_DOUBLE) :: vit_CntrPar_Flp_Kp
        REAL(C_DOUBLE) :: vit_CntrPar_Flp_Ki
        REAL(C_DOUBLE) :: vit_CntrPar_Flp_MaxPit
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_OL_Filename(1024)
        INTEGER(C_INT) :: vit_CntrPar_OL_Mode
        INTEGER(C_INT) :: vit_CntrPar_OL_BP_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_OL_BP_FiltFreq
        INTEGER(C_INT) :: vit_CntrPar_Ind_Breakpoint
        TYPE(C_PTR), VALUE :: vit_CntrPar_Ind_BldPitch_ptr
        INTEGER(C_INT) :: vit_CntrPar_Ind_BldPitch_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_BldPitch_cap
        INTEGER(C_INT) :: vit_CntrPar_Ind_GenTq
        INTEGER(C_INT) :: vit_CntrPar_Ind_YawRate
        INTEGER(C_INT) :: vit_CntrPar_Ind_R_Speed
        INTEGER(C_INT) :: vit_CntrPar_Ind_R_Torque
        INTEGER(C_INT) :: vit_CntrPar_Ind_R_Pitch
        INTEGER(C_INT) :: vit_CntrPar_Ind_Azimuth
        TYPE(C_PTR), VALUE :: vit_CntrPar_RP_Gains_ptr
        INTEGER(C_INT) :: vit_CntrPar_RP_Gains_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_RP_Gains_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_Ind_CableControl_ptr
        INTEGER(C_INT) :: vit_CntrPar_Ind_CableControl_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_CableControl_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_Ind_StructControl_ptr
        INTEGER(C_INT) :: vit_CntrPar_Ind_StructControl_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_Ind_StructControl_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_Breakpoints_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_Breakpoints_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Breakpoints_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_BldPitch1_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_BldPitch1_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BldPitch1_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_BldPitch2_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_BldPitch2_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BldPitch2_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_BldPitch3_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_BldPitch3_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_BldPitch3_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_CableControl_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_CableControl_rows
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_CableControl_cols
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_StructControl_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_StructControl_rows
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_StructControl_cols
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_GenTq_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_GenTq_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_GenTq_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_YawRate_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_YawRate_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_YawRate_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_Azimuth_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_Azimuth_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Azimuth_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_R_Speed_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_R_Speed_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_R_Speed_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_R_Torque_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_R_Torque_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_R_Torque_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_R_Pitch_ptr
        INTEGER(C_INT) :: vit_CntrPar_OL_R_Pitch_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_R_Pitch_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_OL_Channels_ptr
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Channels_rows
        INTEGER(C_INT), VALUE :: vit_CntrPar_OL_Channels_cols
        INTEGER(C_INT) :: vit_CntrPar_PA_Mode
        REAL(C_DOUBLE) :: vit_CntrPar_PA_CornerFreq
        REAL(C_DOUBLE) :: vit_CntrPar_PA_Damping
        INTEGER(C_INT) :: vit_CntrPar_AWC_Mode
        INTEGER(C_INT) :: vit_CntrPar_AWC_NumModes
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_n_ptr
        INTEGER(C_INT) :: vit_CntrPar_AWC_n_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_n_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_harmonic_ptr
        INTEGER(C_INT) :: vit_CntrPar_AWC_harmonic_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_harmonic_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_freq_ptr
        INTEGER(C_INT) :: vit_CntrPar_AWC_freq_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_freq_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_amp_ptr
        INTEGER(C_INT) :: vit_CntrPar_AWC_amp_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_amp_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_clockangle_ptr
        INTEGER(C_INT) :: vit_CntrPar_AWC_clockangle_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_clockangle_cap
        REAL(C_DOUBLE) :: vit_CntrPar_AWC_phaseoffset
        TYPE(C_PTR), VALUE :: vit_CntrPar_AWC_CntrGains_ptr
        INTEGER(C_INT) :: vit_CntrPar_AWC_CntrGains_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_AWC_CntrGains_cap
        INTEGER(C_INT) :: vit_CntrPar_PF_Mode
        TYPE(C_PTR), VALUE :: vit_CntrPar_PF_Offsets_ptr
        INTEGER(C_INT) :: vit_CntrPar_PF_Offsets_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PF_Offsets_cap
        TYPE(C_PTR), VALUE :: vit_CntrPar_PF_TimeStuck_ptr
        INTEGER(C_INT) :: vit_CntrPar_PF_TimeStuck_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_PF_TimeStuck_cap
        INTEGER(C_INT) :: vit_CntrPar_Ext_Mode
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_DLL_FileName(1024)
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_DLL_InFile(1024)
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_DLL_ProcName(1024)
        INTEGER(C_INT) :: vit_CntrPar_ZMQ_Mode
        CHARACTER(KIND=C_CHAR) :: vit_CntrPar_ZMQ_CommAddress(256)
        REAL(C_DOUBLE) :: vit_CntrPar_ZMQ_UpdatePeriod
        INTEGER(C_INT) :: vit_CntrPar_CC_Mode
        INTEGER(C_INT) :: vit_CntrPar_CC_Group_N
        REAL(C_DOUBLE) :: vit_CntrPar_CC_ActTau
        TYPE(C_PTR), VALUE :: vit_CntrPar_CC_GroupIndex_ptr
        INTEGER(C_INT) :: vit_CntrPar_CC_GroupIndex_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_CC_GroupIndex_cap
        INTEGER(C_INT) :: vit_CntrPar_StC_Mode
        INTEGER(C_INT) :: vit_CntrPar_StC_Group_N
        TYPE(C_PTR), VALUE :: vit_CntrPar_StC_GroupIndex_ptr
        INTEGER(C_INT) :: vit_CntrPar_StC_GroupIndex_n
        INTEGER(C_INT), VALUE :: vit_CntrPar_StC_GroupIndex_cap
        REAL(C_DOUBLE) :: vit_CntrPar_PC_RtTq99
        REAL(C_DOUBLE) :: vit_CntrPar_VS_MaxOMTq
        REAL(C_DOUBLE) :: vit_CntrPar_VS_MinOMTq
        INTEGER(C_INT) :: vit_ErrVar_size_avcMSG
        INTEGER(C_INT) :: vit_ErrVar_aviFAIL
        INTEGER(C_INT) :: vit_ErrVar_ErrStat
        CHARACTER(KIND=C_CHAR) :: vit_ErrVar_ErrMsg(*)
        INTEGER(C_INT) :: vit_ErrVar_ErrMsg_n
        INTEGER(C_INT), VALUE :: vit_ErrVar_ErrMsg_cap
        TYPE(LocalVariables) :: vit_local_LocalVar
        CHARACTER(C_CHAR), POINTER :: vit_tmp_LocalVar_ACC_INFILE(:)
        TYPE(WE), POINTER :: vit_nested_LocalVar_WE
        TYPE(FilterParameters), POINTER :: vit_nested_LocalVar_FP
        TYPE(piParams), POINTER :: vit_nested_LocalVar_piP
        TYPE(resParams), POINTER :: vit_nested_LocalVar_resP
        TYPE(rlParams), POINTER :: vit_nested_LocalVar_rlP
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
        INTEGER :: vit_dc_ErrVar_ErrMsg
        TYPE(ErrorVariables) :: vit_local_ErrVar
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
        IF (C_ASSOCIATED(vit_LocalVar_ACC_INFILE_ptr)) &
            CALL C_F_POINTER(vit_LocalVar_ACC_INFILE_ptr, vit_tmp_LocalVar_ACC_INFILE, &
                [MAX(vit_LocalVar_ACC_INFILE_cap, 1)])
        IF (vit_LocalVar_ACC_INFILE_n >= 0) THEN
            ALLOCATE(vit_local_LocalVar%ACC_INFILE(vit_LocalVar_ACC_INFILE_n))
            IF (vit_LocalVar_ACC_INFILE_n > 0 .AND. C_ASSOCIATED(vit_LocalVar_ACC_INFILE_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_F_GenSpdNotch_Ind_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_GenSpdNotch_Ind_ptr, vit_tmp_CntrPar_F_GenSpdNotch_Ind, &
                [MAX(vit_CntrPar_F_GenSpdNotch_Ind_cap, 1)])
        IF (vit_CntrPar_F_GenSpdNotch_Ind_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_GenSpdNotch_Ind(vit_CntrPar_F_GenSpdNotch_Ind_n))
            IF (vit_CntrPar_F_GenSpdNotch_Ind_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_GenSpdNotch_Ind_ptr)) &
                vit_local_CntrPar%F_GenSpdNotch_Ind(1:vit_CntrPar_F_GenSpdNotch_Ind_n) = &
                    vit_tmp_CntrPar_F_GenSpdNotch_Ind(1:vit_CntrPar_F_GenSpdNotch_Ind_n)
        END IF
        vit_local_CntrPar%F_TwrTopNotch_N = vit_CntrPar_F_TwrTopNotch_N
        IF (C_ASSOCIATED(vit_CntrPar_F_TwrTopNotch_Ind_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_TwrTopNotch_Ind_ptr, vit_tmp_CntrPar_F_TwrTopNotch_Ind, &
                [MAX(vit_CntrPar_F_TwrTopNotch_Ind_cap, 1)])
        IF (vit_CntrPar_F_TwrTopNotch_Ind_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_TwrTopNotch_Ind(vit_CntrPar_F_TwrTopNotch_Ind_n))
            IF (vit_CntrPar_F_TwrTopNotch_Ind_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_TwrTopNotch_Ind_ptr)) &
                vit_local_CntrPar%F_TwrTopNotch_Ind(1:vit_CntrPar_F_TwrTopNotch_Ind_n) = &
                    vit_tmp_CntrPar_F_TwrTopNotch_Ind(1:vit_CntrPar_F_TwrTopNotch_Ind_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_F_NotchFreqs_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_NotchFreqs_ptr, vit_tmp_CntrPar_F_NotchFreqs, &
                [MAX(vit_CntrPar_F_NotchFreqs_cap, 1)])
        IF (vit_CntrPar_F_NotchFreqs_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_NotchFreqs(vit_CntrPar_F_NotchFreqs_n))
            IF (vit_CntrPar_F_NotchFreqs_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_NotchFreqs_ptr)) &
                vit_local_CntrPar%F_NotchFreqs(1:vit_CntrPar_F_NotchFreqs_n) = &
                    vit_tmp_CntrPar_F_NotchFreqs(1:vit_CntrPar_F_NotchFreqs_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_F_NotchBetaNum_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_NotchBetaNum_ptr, vit_tmp_CntrPar_F_NotchBetaNum, &
                [MAX(vit_CntrPar_F_NotchBetaNum_cap, 1)])
        IF (vit_CntrPar_F_NotchBetaNum_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_NotchBetaNum(vit_CntrPar_F_NotchBetaNum_n))
            IF (vit_CntrPar_F_NotchBetaNum_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_NotchBetaNum_ptr)) &
                vit_local_CntrPar%F_NotchBetaNum(1:vit_CntrPar_F_NotchBetaNum_n) = &
                    vit_tmp_CntrPar_F_NotchBetaNum(1:vit_CntrPar_F_NotchBetaNum_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_F_NotchBetaDen_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_NotchBetaDen_ptr, vit_tmp_CntrPar_F_NotchBetaDen, &
                [MAX(vit_CntrPar_F_NotchBetaDen_cap, 1)])
        IF (vit_CntrPar_F_NotchBetaDen_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_NotchBetaDen(vit_CntrPar_F_NotchBetaDen_n))
            IF (vit_CntrPar_F_NotchBetaDen_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_NotchBetaDen_ptr)) &
                vit_local_CntrPar%F_NotchBetaDen(1:vit_CntrPar_F_NotchBetaDen_n) = &
                    vit_tmp_CntrPar_F_NotchBetaDen(1:vit_CntrPar_F_NotchBetaDen_n)
        END IF
        vit_local_CntrPar%F_SSCornerFreq = vit_CntrPar_F_SSCornerFreq
        vit_local_CntrPar%F_WECornerFreq = vit_CntrPar_F_WECornerFreq
        IF (C_ASSOCIATED(vit_CntrPar_F_FlCornerFreq_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_FlCornerFreq_ptr, vit_tmp_CntrPar_F_FlCornerFreq, &
                [MAX(vit_CntrPar_F_FlCornerFreq_cap, 1)])
        IF (vit_CntrPar_F_FlCornerFreq_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_FlCornerFreq(vit_CntrPar_F_FlCornerFreq_n))
            IF (vit_CntrPar_F_FlCornerFreq_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_FlCornerFreq_ptr)) &
                vit_local_CntrPar%F_FlCornerFreq(1:vit_CntrPar_F_FlCornerFreq_n) = &
                    vit_tmp_CntrPar_F_FlCornerFreq(1:vit_CntrPar_F_FlCornerFreq_n)
        END IF
        vit_local_CntrPar%F_FlHighPassFreq = vit_CntrPar_F_FlHighPassFreq
        vit_local_CntrPar%F_YawErr = vit_CntrPar_F_YawErr
        IF (C_ASSOCIATED(vit_CntrPar_F_FlpCornerFreq_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_F_FlpCornerFreq_ptr, vit_tmp_CntrPar_F_FlpCornerFreq, &
                [MAX(vit_CntrPar_F_FlpCornerFreq_cap, 1)])
        IF (vit_CntrPar_F_FlpCornerFreq_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%F_FlpCornerFreq(vit_CntrPar_F_FlpCornerFreq_n))
            IF (vit_CntrPar_F_FlpCornerFreq_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_F_FlpCornerFreq_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_IPC_Vramp_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_IPC_Vramp_ptr, vit_tmp_CntrPar_IPC_Vramp, [MAX(vit_CntrPar_IPC_Vramp_cap, 1)])
        IF (vit_CntrPar_IPC_Vramp_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%IPC_Vramp(vit_CntrPar_IPC_Vramp_n))
            IF (vit_CntrPar_IPC_Vramp_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_IPC_Vramp_ptr)) &
                vit_local_CntrPar%IPC_Vramp(1:vit_CntrPar_IPC_Vramp_n) = &
                    vit_tmp_CntrPar_IPC_Vramp(1:vit_CntrPar_IPC_Vramp_n)
        END IF
        vit_local_CntrPar%IPC_IntSat = vit_CntrPar_IPC_IntSat
        vit_local_CntrPar%IPC_SatMode = vit_CntrPar_IPC_SatMode
        IF (C_ASSOCIATED(vit_CntrPar_IPC_KP_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_IPC_KP_ptr, vit_tmp_CntrPar_IPC_KP, [MAX(vit_CntrPar_IPC_KP_cap, 1)])
        IF (vit_CntrPar_IPC_KP_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%IPC_KP(vit_CntrPar_IPC_KP_n))
            IF (vit_CntrPar_IPC_KP_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_IPC_KP_ptr)) &
                vit_local_CntrPar%IPC_KP(1:vit_CntrPar_IPC_KP_n) = vit_tmp_CntrPar_IPC_KP(1:vit_CntrPar_IPC_KP_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_IPC_KI_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_IPC_KI_ptr, vit_tmp_CntrPar_IPC_KI, [MAX(vit_CntrPar_IPC_KI_cap, 1)])
        IF (vit_CntrPar_IPC_KI_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%IPC_KI(vit_CntrPar_IPC_KI_n))
            IF (vit_CntrPar_IPC_KI_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_IPC_KI_ptr)) &
                vit_local_CntrPar%IPC_KI(1:vit_CntrPar_IPC_KI_n) = vit_tmp_CntrPar_IPC_KI(1:vit_CntrPar_IPC_KI_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_IPC_aziOffset_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_IPC_aziOffset_ptr, vit_tmp_CntrPar_IPC_aziOffset, &
                [MAX(vit_CntrPar_IPC_aziOffset_cap, 1)])
        IF (vit_CntrPar_IPC_aziOffset_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%IPC_aziOffset(vit_CntrPar_IPC_aziOffset_n))
            IF (vit_CntrPar_IPC_aziOffset_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_IPC_aziOffset_ptr)) &
                vit_local_CntrPar%IPC_aziOffset(1:vit_CntrPar_IPC_aziOffset_n) = &
                    vit_tmp_CntrPar_IPC_aziOffset(1:vit_CntrPar_IPC_aziOffset_n)
        END IF
        vit_local_CntrPar%IPC_CornerFreqAct = vit_CntrPar_IPC_CornerFreqAct
        vit_local_CntrPar%PC_ControlMode = vit_CntrPar_PC_ControlMode
        vit_local_CntrPar%PC_GS_n = vit_CntrPar_PC_GS_n
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_angles_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PC_GS_angles_ptr, vit_tmp_CntrPar_PC_GS_angles, &
                [MAX(vit_CntrPar_PC_GS_angles_cap, 1)])
        IF (vit_CntrPar_PC_GS_angles_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_angles(vit_CntrPar_PC_GS_angles_n))
            IF (vit_CntrPar_PC_GS_angles_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PC_GS_angles_ptr)) &
                vit_local_CntrPar%PC_GS_angles(1:vit_CntrPar_PC_GS_angles_n) = &
                    vit_tmp_CntrPar_PC_GS_angles(1:vit_CntrPar_PC_GS_angles_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_KP_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PC_GS_KP_ptr, vit_tmp_CntrPar_PC_GS_KP, [MAX(vit_CntrPar_PC_GS_KP_cap, 1)])
        IF (vit_CntrPar_PC_GS_KP_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_KP(vit_CntrPar_PC_GS_KP_n))
            IF (vit_CntrPar_PC_GS_KP_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PC_GS_KP_ptr)) &
                vit_local_CntrPar%PC_GS_KP(1:vit_CntrPar_PC_GS_KP_n) = &
                    vit_tmp_CntrPar_PC_GS_KP(1:vit_CntrPar_PC_GS_KP_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_KI_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PC_GS_KI_ptr, vit_tmp_CntrPar_PC_GS_KI, [MAX(vit_CntrPar_PC_GS_KI_cap, 1)])
        IF (vit_CntrPar_PC_GS_KI_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_KI(vit_CntrPar_PC_GS_KI_n))
            IF (vit_CntrPar_PC_GS_KI_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PC_GS_KI_ptr)) &
                vit_local_CntrPar%PC_GS_KI(1:vit_CntrPar_PC_GS_KI_n) = &
                    vit_tmp_CntrPar_PC_GS_KI(1:vit_CntrPar_PC_GS_KI_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_KD_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PC_GS_KD_ptr, vit_tmp_CntrPar_PC_GS_KD, [MAX(vit_CntrPar_PC_GS_KD_cap, 1)])
        IF (vit_CntrPar_PC_GS_KD_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_KD(vit_CntrPar_PC_GS_KD_n))
            IF (vit_CntrPar_PC_GS_KD_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PC_GS_KD_ptr)) &
                vit_local_CntrPar%PC_GS_KD(1:vit_CntrPar_PC_GS_KD_n) = &
                    vit_tmp_CntrPar_PC_GS_KD(1:vit_CntrPar_PC_GS_KD_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PC_GS_TF_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PC_GS_TF_ptr, vit_tmp_CntrPar_PC_GS_TF, [MAX(vit_CntrPar_PC_GS_TF_cap, 1)])
        IF (vit_CntrPar_PC_GS_TF_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PC_GS_TF(vit_CntrPar_PC_GS_TF_n))
            IF (vit_CntrPar_PC_GS_TF_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PC_GS_TF_ptr)) &
                vit_local_CntrPar%PC_GS_TF(1:vit_CntrPar_PC_GS_TF_n) = &
                    vit_tmp_CntrPar_PC_GS_TF(1:vit_CntrPar_PC_GS_TF_n)
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
        IF (C_ASSOCIATED(vit_CntrPar_VS_KP_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_VS_KP_ptr, vit_tmp_CntrPar_VS_KP, [MAX(vit_CntrPar_VS_KP_cap, 1)])
        IF (vit_CntrPar_VS_KP_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%VS_KP(vit_CntrPar_VS_KP_n))
            IF (vit_CntrPar_VS_KP_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_VS_KP_ptr)) &
                vit_local_CntrPar%VS_KP(1:vit_CntrPar_VS_KP_n) = vit_tmp_CntrPar_VS_KP(1:vit_CntrPar_VS_KP_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_VS_KI_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_VS_KI_ptr, vit_tmp_CntrPar_VS_KI, [MAX(vit_CntrPar_VS_KI_cap, 1)])
        IF (vit_CntrPar_VS_KI_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%VS_KI(vit_CntrPar_VS_KI_n))
            IF (vit_CntrPar_VS_KI_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_VS_KI_ptr)) &
                vit_local_CntrPar%VS_KI(1:vit_CntrPar_VS_KI_n) = vit_tmp_CntrPar_VS_KI(1:vit_CntrPar_VS_KI_n)
        END IF
        vit_local_CntrPar%VS_TSRopt = vit_CntrPar_VS_TSRopt
        vit_local_CntrPar%VS_FBP_n = vit_CntrPar_VS_FBP_n
        IF (C_ASSOCIATED(vit_CntrPar_VS_FBP_U_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_VS_FBP_U_ptr, vit_tmp_CntrPar_VS_FBP_U, [MAX(vit_CntrPar_VS_FBP_U_cap, 1)])
        IF (vit_CntrPar_VS_FBP_U_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%VS_FBP_U(vit_CntrPar_VS_FBP_U_n))
            IF (vit_CntrPar_VS_FBP_U_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_VS_FBP_U_ptr)) &
                vit_local_CntrPar%VS_FBP_U(1:vit_CntrPar_VS_FBP_U_n) = &
                    vit_tmp_CntrPar_VS_FBP_U(1:vit_CntrPar_VS_FBP_U_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_VS_FBP_Omega_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_VS_FBP_Omega_ptr, vit_tmp_CntrPar_VS_FBP_Omega, &
                [MAX(vit_CntrPar_VS_FBP_Omega_cap, 1)])
        IF (vit_CntrPar_VS_FBP_Omega_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%VS_FBP_Omega(vit_CntrPar_VS_FBP_Omega_n))
            IF (vit_CntrPar_VS_FBP_Omega_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_VS_FBP_Omega_ptr)) &
                vit_local_CntrPar%VS_FBP_Omega(1:vit_CntrPar_VS_FBP_Omega_n) = &
                    vit_tmp_CntrPar_VS_FBP_Omega(1:vit_CntrPar_VS_FBP_Omega_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_VS_FBP_Tau_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_VS_FBP_Tau_ptr, vit_tmp_CntrPar_VS_FBP_Tau, [MAX(vit_CntrPar_VS_FBP_Tau_cap, &
                1)])
        IF (vit_CntrPar_VS_FBP_Tau_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%VS_FBP_Tau(vit_CntrPar_VS_FBP_Tau_n))
            IF (vit_CntrPar_VS_FBP_Tau_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_VS_FBP_Tau_ptr)) &
                vit_local_CntrPar%VS_FBP_Tau(1:vit_CntrPar_VS_FBP_Tau_n) = &
                    vit_tmp_CntrPar_VS_FBP_Tau(1:vit_CntrPar_VS_FBP_Tau_n)
        END IF
        vit_local_CntrPar%SS_Mode = vit_CntrPar_SS_Mode
        vit_local_CntrPar%SS_VSGain = vit_CntrPar_SS_VSGain
        vit_local_CntrPar%SS_PCGain = vit_CntrPar_SS_PCGain
        vit_local_CntrPar%PRC_Mode = vit_CntrPar_PRC_Mode
        vit_local_CntrPar%PRC_Comm = vit_CntrPar_PRC_Comm
        IF (C_ASSOCIATED(vit_CntrPar_PRC_WindSpeeds_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PRC_WindSpeeds_ptr, vit_tmp_CntrPar_PRC_WindSpeeds, &
                [MAX(vit_CntrPar_PRC_WindSpeeds_cap, 1)])
        IF (vit_CntrPar_PRC_WindSpeeds_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PRC_WindSpeeds(vit_CntrPar_PRC_WindSpeeds_n))
            IF (vit_CntrPar_PRC_WindSpeeds_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PRC_WindSpeeds_ptr)) &
                vit_local_CntrPar%PRC_WindSpeeds(1:vit_CntrPar_PRC_WindSpeeds_n) = &
                    vit_tmp_CntrPar_PRC_WindSpeeds(1:vit_CntrPar_PRC_WindSpeeds_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PRC_GenSpeeds_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PRC_GenSpeeds_ptr, vit_tmp_CntrPar_PRC_GenSpeeds, &
                [MAX(vit_CntrPar_PRC_GenSpeeds_cap, 1)])
        IF (vit_CntrPar_PRC_GenSpeeds_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PRC_GenSpeeds(vit_CntrPar_PRC_GenSpeeds_n))
            IF (vit_CntrPar_PRC_GenSpeeds_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PRC_GenSpeeds_ptr)) &
                vit_local_CntrPar%PRC_GenSpeeds(1:vit_CntrPar_PRC_GenSpeeds_n) = &
                    vit_tmp_CntrPar_PRC_GenSpeeds(1:vit_CntrPar_PRC_GenSpeeds_n)
        END IF
        vit_local_CntrPar%PRC_n = vit_CntrPar_PRC_n
        vit_local_CntrPar%PRC_LPF_Freq = vit_CntrPar_PRC_LPF_Freq
        vit_local_CntrPar%PRC_R_Torque = vit_CntrPar_PRC_R_Torque
        vit_local_CntrPar%PRC_R_Speed = vit_CntrPar_PRC_R_Speed
        vit_local_CntrPar%PRC_R_Pitch = vit_CntrPar_PRC_R_Pitch
        vit_local_CntrPar%PRC_Table_n = vit_CntrPar_PRC_Table_n
        IF (C_ASSOCIATED(vit_CntrPar_PRC_Pitch_Table_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PRC_Pitch_Table_ptr, vit_tmp_CntrPar_PRC_Pitch_Table, &
                [MAX(vit_CntrPar_PRC_Pitch_Table_cap, 1)])
        IF (vit_CntrPar_PRC_Pitch_Table_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PRC_Pitch_Table(vit_CntrPar_PRC_Pitch_Table_n))
            IF (vit_CntrPar_PRC_Pitch_Table_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PRC_Pitch_Table_ptr)) &
                vit_local_CntrPar%PRC_Pitch_Table(1:vit_CntrPar_PRC_Pitch_Table_n) = &
                    vit_tmp_CntrPar_PRC_Pitch_Table(1:vit_CntrPar_PRC_Pitch_Table_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PRC_R_Table_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PRC_R_Table_ptr, vit_tmp_CntrPar_PRC_R_Table, &
                [MAX(vit_CntrPar_PRC_R_Table_cap, 1)])
        IF (vit_CntrPar_PRC_R_Table_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PRC_R_Table(vit_CntrPar_PRC_R_Table_n))
            IF (vit_CntrPar_PRC_R_Table_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PRC_R_Table_ptr)) &
                vit_local_CntrPar%PRC_R_Table(1:vit_CntrPar_PRC_R_Table_n) = &
                    vit_tmp_CntrPar_PRC_R_Table(1:vit_CntrPar_PRC_R_Table_n)
        END IF
        vit_local_CntrPar%WE_Mode = vit_CntrPar_WE_Mode
        vit_local_CntrPar%WE_BladeRadius = vit_CntrPar_WE_BladeRadius
        vit_local_CntrPar%WE_CP_n = vit_CntrPar_WE_CP_n
        IF (C_ASSOCIATED(vit_CntrPar_WE_CP_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_WE_CP_ptr, vit_tmp_CntrPar_WE_CP, [MAX(vit_CntrPar_WE_CP_cap, 1)])
        IF (vit_CntrPar_WE_CP_n_v2 >= 0) THEN
            ALLOCATE(vit_local_CntrPar%WE_CP(vit_CntrPar_WE_CP_n_v2))
            IF (vit_CntrPar_WE_CP_n_v2 > 0 .AND. C_ASSOCIATED(vit_CntrPar_WE_CP_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_PerfTableSize_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PerfTableSize_ptr, vit_tmp_CntrPar_PerfTableSize, &
                [MAX(vit_CntrPar_PerfTableSize_cap, 1)])
        IF (vit_CntrPar_PerfTableSize_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PerfTableSize(vit_CntrPar_PerfTableSize_n))
            IF (vit_CntrPar_PerfTableSize_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PerfTableSize_ptr)) &
                vit_local_CntrPar%PerfTableSize(1:vit_CntrPar_PerfTableSize_n) = &
                    vit_tmp_CntrPar_PerfTableSize(1:vit_CntrPar_PerfTableSize_n)
        END IF
        vit_local_CntrPar%WE_FOPoles_N = vit_CntrPar_WE_FOPoles_N
        IF (C_ASSOCIATED(vit_CntrPar_WE_FOPoles_v_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_WE_FOPoles_v_ptr, vit_tmp_CntrPar_WE_FOPoles_v, &
                [MAX(vit_CntrPar_WE_FOPoles_v_cap, 1)])
        IF (vit_CntrPar_WE_FOPoles_v_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%WE_FOPoles_v(vit_CntrPar_WE_FOPoles_v_n))
            IF (vit_CntrPar_WE_FOPoles_v_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_WE_FOPoles_v_ptr)) &
                vit_local_CntrPar%WE_FOPoles_v(1:vit_CntrPar_WE_FOPoles_v_n) = &
                    vit_tmp_CntrPar_WE_FOPoles_v(1:vit_CntrPar_WE_FOPoles_v_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_WE_FOPoles_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_WE_FOPoles_ptr, vit_tmp_CntrPar_WE_FOPoles, [MAX(vit_CntrPar_WE_FOPoles_cap, &
                1)])
        IF (vit_CntrPar_WE_FOPoles_n_v2 >= 0) THEN
            ALLOCATE(vit_local_CntrPar%WE_FOPoles(vit_CntrPar_WE_FOPoles_n_v2))
            IF (vit_CntrPar_WE_FOPoles_n_v2 > 0 .AND. C_ASSOCIATED(vit_CntrPar_WE_FOPoles_ptr)) &
                vit_local_CntrPar%WE_FOPoles(1:vit_CntrPar_WE_FOPoles_n_v2) = &
                    vit_tmp_CntrPar_WE_FOPoles(1:vit_CntrPar_WE_FOPoles_n_v2)
        END IF
        vit_local_CntrPar%Y_ControlMode = vit_CntrPar_Y_ControlMode
        vit_local_CntrPar%Y_uSwitch = vit_CntrPar_Y_uSwitch
        IF (C_ASSOCIATED(vit_CntrPar_Y_ErrThresh_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_Y_ErrThresh_ptr, vit_tmp_CntrPar_Y_ErrThresh, &
                [MAX(vit_CntrPar_Y_ErrThresh_cap, 1)])
        IF (vit_CntrPar_Y_ErrThresh_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%Y_ErrThresh(vit_CntrPar_Y_ErrThresh_n))
            IF (vit_CntrPar_Y_ErrThresh_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_Y_ErrThresh_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_PS_WindSpeeds_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PS_WindSpeeds_ptr, vit_tmp_CntrPar_PS_WindSpeeds, &
                [MAX(vit_CntrPar_PS_WindSpeeds_cap, 1)])
        IF (vit_CntrPar_PS_WindSpeeds_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PS_WindSpeeds(vit_CntrPar_PS_WindSpeeds_n))
            IF (vit_CntrPar_PS_WindSpeeds_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PS_WindSpeeds_ptr)) &
                vit_local_CntrPar%PS_WindSpeeds(1:vit_CntrPar_PS_WindSpeeds_n) = &
                    vit_tmp_CntrPar_PS_WindSpeeds(1:vit_CntrPar_PS_WindSpeeds_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PS_BldPitchMin_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PS_BldPitchMin_ptr, vit_tmp_CntrPar_PS_BldPitchMin, &
                [MAX(vit_CntrPar_PS_BldPitchMin_cap, 1)])
        IF (vit_CntrPar_PS_BldPitchMin_n_v2 >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PS_BldPitchMin(vit_CntrPar_PS_BldPitchMin_n_v2))
            IF (vit_CntrPar_PS_BldPitchMin_n_v2 > 0 .AND. C_ASSOCIATED(vit_CntrPar_PS_BldPitchMin_ptr)) &
                vit_local_CntrPar%PS_BldPitchMin(1:vit_CntrPar_PS_BldPitchMin_n_v2) = &
                    vit_tmp_CntrPar_PS_BldPitchMin(1:vit_CntrPar_PS_BldPitchMin_n_v2)
        END IF
        vit_local_CntrPar%SU_Mode = vit_CntrPar_SU_Mode
        vit_local_CntrPar%SU_StartTime = vit_CntrPar_SU_StartTime
        vit_local_CntrPar%SU_FW_MinDuration = vit_CntrPar_SU_FW_MinDuration
        vit_local_CntrPar%SU_RotorSpeedThresh = vit_CntrPar_SU_RotorSpeedThresh
        vit_local_CntrPar%SU_RotorSpeedCornerFreq = vit_CntrPar_SU_RotorSpeedCornerFreq
        vit_local_CntrPar%SU_LoadStages_N = vit_CntrPar_SU_LoadStages_N
        IF (C_ASSOCIATED(vit_CntrPar_SU_LoadStages_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SU_LoadStages_ptr, vit_tmp_CntrPar_SU_LoadStages, &
                [MAX(vit_CntrPar_SU_LoadStages_cap, 1)])
        IF (vit_CntrPar_SU_LoadStages_n_v2 >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SU_LoadStages(vit_CntrPar_SU_LoadStages_n_v2))
            IF (vit_CntrPar_SU_LoadStages_n_v2 > 0 .AND. C_ASSOCIATED(vit_CntrPar_SU_LoadStages_ptr)) &
                vit_local_CntrPar%SU_LoadStages(1:vit_CntrPar_SU_LoadStages_n_v2) = &
                    vit_tmp_CntrPar_SU_LoadStages(1:vit_CntrPar_SU_LoadStages_n_v2)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SU_LoadRampDuration_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SU_LoadRampDuration_ptr, vit_tmp_CntrPar_SU_LoadRampDuration, &
                [MAX(vit_CntrPar_SU_LoadRampDuration_cap, 1)])
        IF (vit_CntrPar_SU_LoadRampDuration_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SU_LoadRampDuration(vit_CntrPar_SU_LoadRampDuration_n))
            IF (vit_CntrPar_SU_LoadRampDuration_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_SU_LoadRampDuration_ptr)) &
                vit_local_CntrPar%SU_LoadRampDuration(1:vit_CntrPar_SU_LoadRampDuration_n) = &
                    vit_tmp_CntrPar_SU_LoadRampDuration(1:vit_CntrPar_SU_LoadRampDuration_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SU_LoadHoldDuration_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SU_LoadHoldDuration_ptr, vit_tmp_CntrPar_SU_LoadHoldDuration, &
                [MAX(vit_CntrPar_SU_LoadHoldDuration_cap, 1)])
        IF (vit_CntrPar_SU_LoadHoldDuration_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SU_LoadHoldDuration(vit_CntrPar_SU_LoadHoldDuration_n))
            IF (vit_CntrPar_SU_LoadHoldDuration_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_SU_LoadHoldDuration_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_SD_MaxTorqueRate_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SD_MaxTorqueRate_ptr, vit_tmp_CntrPar_SD_MaxTorqueRate, &
                [MAX(vit_CntrPar_SD_MaxTorqueRate_cap, 1)])
        IF (vit_CntrPar_SD_MaxTorqueRate_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SD_MaxTorqueRate(vit_CntrPar_SD_MaxTorqueRate_n))
            IF (vit_CntrPar_SD_MaxTorqueRate_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_SD_MaxTorqueRate_ptr)) &
                vit_local_CntrPar%SD_MaxTorqueRate(1:vit_CntrPar_SD_MaxTorqueRate_n) = &
                    vit_tmp_CntrPar_SD_MaxTorqueRate(1:vit_CntrPar_SD_MaxTorqueRate_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SD_MaxPitchRate_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SD_MaxPitchRate_ptr, vit_tmp_CntrPar_SD_MaxPitchRate, &
                [MAX(vit_CntrPar_SD_MaxPitchRate_cap, 1)])
        IF (vit_CntrPar_SD_MaxPitchRate_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SD_MaxPitchRate(vit_CntrPar_SD_MaxPitchRate_n))
            IF (vit_CntrPar_SD_MaxPitchRate_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_SD_MaxPitchRate_ptr)) &
                vit_local_CntrPar%SD_MaxPitchRate(1:vit_CntrPar_SD_MaxPitchRate_n) = &
                    vit_tmp_CntrPar_SD_MaxPitchRate(1:vit_CntrPar_SD_MaxPitchRate_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SD_StagePitch_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SD_StagePitch_ptr, vit_tmp_CntrPar_SD_StagePitch, &
                [MAX(vit_CntrPar_SD_StagePitch_cap, 1)])
        IF (vit_CntrPar_SD_StagePitch_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SD_StagePitch(vit_CntrPar_SD_StagePitch_n))
            IF (vit_CntrPar_SD_StagePitch_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_SD_StagePitch_ptr)) &
                vit_local_CntrPar%SD_StagePitch(1:vit_CntrPar_SD_StagePitch_n) = &
                    vit_tmp_CntrPar_SD_StagePitch(1:vit_CntrPar_SD_StagePitch_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_SD_StageTime_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_SD_StageTime_ptr, vit_tmp_CntrPar_SD_StageTime, &
                [MAX(vit_CntrPar_SD_StageTime_cap, 1)])
        IF (vit_CntrPar_SD_StageTime_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%SD_StageTime(vit_CntrPar_SD_StageTime_n))
            IF (vit_CntrPar_SD_StageTime_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_SD_StageTime_ptr)) &
                vit_local_CntrPar%SD_StageTime(1:vit_CntrPar_SD_StageTime_n) = &
                    vit_tmp_CntrPar_SD_StageTime(1:vit_CntrPar_SD_StageTime_n)
        END IF
        vit_local_CntrPar%SD_Stage_N = vit_CntrPar_SD_Stage_N
        vit_local_CntrPar%Fl_Mode = vit_CntrPar_Fl_Mode
        vit_local_CntrPar%Fl_n = vit_CntrPar_Fl_n
        IF (C_ASSOCIATED(vit_CntrPar_Fl_Kp_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_Fl_Kp_ptr, vit_tmp_CntrPar_Fl_Kp, [MAX(vit_CntrPar_Fl_Kp_cap, 1)])
        IF (vit_CntrPar_Fl_Kp_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%Fl_Kp(vit_CntrPar_Fl_Kp_n))
            IF (vit_CntrPar_Fl_Kp_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_Fl_Kp_ptr)) &
                vit_local_CntrPar%Fl_Kp(1:vit_CntrPar_Fl_Kp_n) = vit_tmp_CntrPar_Fl_Kp(1:vit_CntrPar_Fl_Kp_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_Fl_U_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_Fl_U_ptr, vit_tmp_CntrPar_Fl_U, [MAX(vit_CntrPar_Fl_U_cap, 1)])
        IF (vit_CntrPar_Fl_U_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%Fl_U(vit_CntrPar_Fl_U_n))
            IF (vit_CntrPar_Fl_U_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_Fl_U_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_Ind_BldPitch_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_Ind_BldPitch_ptr, vit_tmp_CntrPar_Ind_BldPitch, &
                [MAX(vit_CntrPar_Ind_BldPitch_cap, 1)])
        IF (vit_CntrPar_Ind_BldPitch_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%Ind_BldPitch(vit_CntrPar_Ind_BldPitch_n))
            IF (vit_CntrPar_Ind_BldPitch_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_Ind_BldPitch_ptr)) &
                vit_local_CntrPar%Ind_BldPitch(1:vit_CntrPar_Ind_BldPitch_n) = &
                    vit_tmp_CntrPar_Ind_BldPitch(1:vit_CntrPar_Ind_BldPitch_n)
        END IF
        vit_local_CntrPar%Ind_GenTq = vit_CntrPar_Ind_GenTq
        vit_local_CntrPar%Ind_YawRate = vit_CntrPar_Ind_YawRate
        vit_local_CntrPar%Ind_R_Speed = vit_CntrPar_Ind_R_Speed
        vit_local_CntrPar%Ind_R_Torque = vit_CntrPar_Ind_R_Torque
        vit_local_CntrPar%Ind_R_Pitch = vit_CntrPar_Ind_R_Pitch
        vit_local_CntrPar%Ind_Azimuth = vit_CntrPar_Ind_Azimuth
        IF (C_ASSOCIATED(vit_CntrPar_RP_Gains_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_RP_Gains_ptr, vit_tmp_CntrPar_RP_Gains, [MAX(vit_CntrPar_RP_Gains_cap, 1)])
        IF (vit_CntrPar_RP_Gains_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%RP_Gains(vit_CntrPar_RP_Gains_n))
            IF (vit_CntrPar_RP_Gains_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_RP_Gains_ptr)) &
                vit_local_CntrPar%RP_Gains(1:vit_CntrPar_RP_Gains_n) = &
                    vit_tmp_CntrPar_RP_Gains(1:vit_CntrPar_RP_Gains_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_Ind_CableControl_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_Ind_CableControl_ptr, vit_tmp_CntrPar_Ind_CableControl, &
                [MAX(vit_CntrPar_Ind_CableControl_cap, 1)])
        IF (vit_CntrPar_Ind_CableControl_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%Ind_CableControl(vit_CntrPar_Ind_CableControl_n))
            IF (vit_CntrPar_Ind_CableControl_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_Ind_CableControl_ptr)) &
                vit_local_CntrPar%Ind_CableControl(1:vit_CntrPar_Ind_CableControl_n) = &
                    vit_tmp_CntrPar_Ind_CableControl(1:vit_CntrPar_Ind_CableControl_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_Ind_StructControl_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_Ind_StructControl_ptr, vit_tmp_CntrPar_Ind_StructControl, &
                [MAX(vit_CntrPar_Ind_StructControl_cap, 1)])
        IF (vit_CntrPar_Ind_StructControl_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%Ind_StructControl(vit_CntrPar_Ind_StructControl_n))
            IF (vit_CntrPar_Ind_StructControl_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_Ind_StructControl_ptr)) &
                vit_local_CntrPar%Ind_StructControl(1:vit_CntrPar_Ind_StructControl_n) = &
                    vit_tmp_CntrPar_Ind_StructControl(1:vit_CntrPar_Ind_StructControl_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_Breakpoints_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_Breakpoints_ptr, vit_tmp_CntrPar_OL_Breakpoints, &
                [MAX(vit_CntrPar_OL_Breakpoints_cap, 1)])
        IF (vit_CntrPar_OL_Breakpoints_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_Breakpoints(vit_CntrPar_OL_Breakpoints_n))
            IF (vit_CntrPar_OL_Breakpoints_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_Breakpoints_ptr)) &
                vit_local_CntrPar%OL_Breakpoints(1:vit_CntrPar_OL_Breakpoints_n) = &
                    vit_tmp_CntrPar_OL_Breakpoints(1:vit_CntrPar_OL_Breakpoints_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_BldPitch1_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_BldPitch1_ptr, vit_tmp_CntrPar_OL_BldPitch1, &
                [MAX(vit_CntrPar_OL_BldPitch1_cap, 1)])
        IF (vit_CntrPar_OL_BldPitch1_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_BldPitch1(vit_CntrPar_OL_BldPitch1_n))
            IF (vit_CntrPar_OL_BldPitch1_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_BldPitch1_ptr)) &
                vit_local_CntrPar%OL_BldPitch1(1:vit_CntrPar_OL_BldPitch1_n) = &
                    vit_tmp_CntrPar_OL_BldPitch1(1:vit_CntrPar_OL_BldPitch1_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_BldPitch2_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_BldPitch2_ptr, vit_tmp_CntrPar_OL_BldPitch2, &
                [MAX(vit_CntrPar_OL_BldPitch2_cap, 1)])
        IF (vit_CntrPar_OL_BldPitch2_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_BldPitch2(vit_CntrPar_OL_BldPitch2_n))
            IF (vit_CntrPar_OL_BldPitch2_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_BldPitch2_ptr)) &
                vit_local_CntrPar%OL_BldPitch2(1:vit_CntrPar_OL_BldPitch2_n) = &
                    vit_tmp_CntrPar_OL_BldPitch2(1:vit_CntrPar_OL_BldPitch2_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_BldPitch3_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_BldPitch3_ptr, vit_tmp_CntrPar_OL_BldPitch3, &
                [MAX(vit_CntrPar_OL_BldPitch3_cap, 1)])
        IF (vit_CntrPar_OL_BldPitch3_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_BldPitch3(vit_CntrPar_OL_BldPitch3_n))
            IF (vit_CntrPar_OL_BldPitch3_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_BldPitch3_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_OL_GenTq_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_GenTq_ptr, vit_tmp_CntrPar_OL_GenTq, [MAX(vit_CntrPar_OL_GenTq_cap, 1)])
        IF (vit_CntrPar_OL_GenTq_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_GenTq(vit_CntrPar_OL_GenTq_n))
            IF (vit_CntrPar_OL_GenTq_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_GenTq_ptr)) &
                vit_local_CntrPar%OL_GenTq(1:vit_CntrPar_OL_GenTq_n) = &
                    vit_tmp_CntrPar_OL_GenTq(1:vit_CntrPar_OL_GenTq_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_YawRate_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_YawRate_ptr, vit_tmp_CntrPar_OL_YawRate, [MAX(vit_CntrPar_OL_YawRate_cap, &
                1)])
        IF (vit_CntrPar_OL_YawRate_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_YawRate(vit_CntrPar_OL_YawRate_n))
            IF (vit_CntrPar_OL_YawRate_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_YawRate_ptr)) &
                vit_local_CntrPar%OL_YawRate(1:vit_CntrPar_OL_YawRate_n) = &
                    vit_tmp_CntrPar_OL_YawRate(1:vit_CntrPar_OL_YawRate_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_Azimuth_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_Azimuth_ptr, vit_tmp_CntrPar_OL_Azimuth, [MAX(vit_CntrPar_OL_Azimuth_cap, &
                1)])
        IF (vit_CntrPar_OL_Azimuth_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_Azimuth(vit_CntrPar_OL_Azimuth_n))
            IF (vit_CntrPar_OL_Azimuth_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_Azimuth_ptr)) &
                vit_local_CntrPar%OL_Azimuth(1:vit_CntrPar_OL_Azimuth_n) = &
                    vit_tmp_CntrPar_OL_Azimuth(1:vit_CntrPar_OL_Azimuth_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_R_Speed_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_R_Speed_ptr, vit_tmp_CntrPar_OL_R_Speed, [MAX(vit_CntrPar_OL_R_Speed_cap, &
                1)])
        IF (vit_CntrPar_OL_R_Speed_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_R_Speed(vit_CntrPar_OL_R_Speed_n))
            IF (vit_CntrPar_OL_R_Speed_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_R_Speed_ptr)) &
                vit_local_CntrPar%OL_R_Speed(1:vit_CntrPar_OL_R_Speed_n) = &
                    vit_tmp_CntrPar_OL_R_Speed(1:vit_CntrPar_OL_R_Speed_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_R_Torque_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_R_Torque_ptr, vit_tmp_CntrPar_OL_R_Torque, &
                [MAX(vit_CntrPar_OL_R_Torque_cap, 1)])
        IF (vit_CntrPar_OL_R_Torque_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_R_Torque(vit_CntrPar_OL_R_Torque_n))
            IF (vit_CntrPar_OL_R_Torque_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_R_Torque_ptr)) &
                vit_local_CntrPar%OL_R_Torque(1:vit_CntrPar_OL_R_Torque_n) = &
                    vit_tmp_CntrPar_OL_R_Torque(1:vit_CntrPar_OL_R_Torque_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_OL_R_Pitch_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_OL_R_Pitch_ptr, vit_tmp_CntrPar_OL_R_Pitch, [MAX(vit_CntrPar_OL_R_Pitch_cap, &
                1)])
        IF (vit_CntrPar_OL_R_Pitch_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%OL_R_Pitch(vit_CntrPar_OL_R_Pitch_n))
            IF (vit_CntrPar_OL_R_Pitch_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_OL_R_Pitch_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_AWC_n_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_AWC_n_ptr, vit_tmp_CntrPar_AWC_n, [MAX(vit_CntrPar_AWC_n_cap, 1)])
        IF (vit_CntrPar_AWC_n_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%AWC_n(vit_CntrPar_AWC_n_n))
            IF (vit_CntrPar_AWC_n_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_AWC_n_ptr)) &
                vit_local_CntrPar%AWC_n(1:vit_CntrPar_AWC_n_n) = vit_tmp_CntrPar_AWC_n(1:vit_CntrPar_AWC_n_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_harmonic_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_AWC_harmonic_ptr, vit_tmp_CntrPar_AWC_harmonic, &
                [MAX(vit_CntrPar_AWC_harmonic_cap, 1)])
        IF (vit_CntrPar_AWC_harmonic_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%AWC_harmonic(vit_CntrPar_AWC_harmonic_n))
            IF (vit_CntrPar_AWC_harmonic_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_AWC_harmonic_ptr)) &
                vit_local_CntrPar%AWC_harmonic(1:vit_CntrPar_AWC_harmonic_n) = &
                    vit_tmp_CntrPar_AWC_harmonic(1:vit_CntrPar_AWC_harmonic_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_freq_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_AWC_freq_ptr, vit_tmp_CntrPar_AWC_freq, [MAX(vit_CntrPar_AWC_freq_cap, 1)])
        IF (vit_CntrPar_AWC_freq_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%AWC_freq(vit_CntrPar_AWC_freq_n))
            IF (vit_CntrPar_AWC_freq_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_AWC_freq_ptr)) &
                vit_local_CntrPar%AWC_freq(1:vit_CntrPar_AWC_freq_n) = &
                    vit_tmp_CntrPar_AWC_freq(1:vit_CntrPar_AWC_freq_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_amp_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_AWC_amp_ptr, vit_tmp_CntrPar_AWC_amp, [MAX(vit_CntrPar_AWC_amp_cap, 1)])
        IF (vit_CntrPar_AWC_amp_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%AWC_amp(vit_CntrPar_AWC_amp_n))
            IF (vit_CntrPar_AWC_amp_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_AWC_amp_ptr)) &
                vit_local_CntrPar%AWC_amp(1:vit_CntrPar_AWC_amp_n) = vit_tmp_CntrPar_AWC_amp(1:vit_CntrPar_AWC_amp_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_AWC_clockangle_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_AWC_clockangle_ptr, vit_tmp_CntrPar_AWC_clockangle, &
                [MAX(vit_CntrPar_AWC_clockangle_cap, 1)])
        IF (vit_CntrPar_AWC_clockangle_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%AWC_clockangle(vit_CntrPar_AWC_clockangle_n))
            IF (vit_CntrPar_AWC_clockangle_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_AWC_clockangle_ptr)) &
                vit_local_CntrPar%AWC_clockangle(1:vit_CntrPar_AWC_clockangle_n) = &
                    vit_tmp_CntrPar_AWC_clockangle(1:vit_CntrPar_AWC_clockangle_n)
        END IF
        vit_local_CntrPar%AWC_phaseoffset = vit_CntrPar_AWC_phaseoffset
        IF (C_ASSOCIATED(vit_CntrPar_AWC_CntrGains_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_AWC_CntrGains_ptr, vit_tmp_CntrPar_AWC_CntrGains, &
                [MAX(vit_CntrPar_AWC_CntrGains_cap, 1)])
        IF (vit_CntrPar_AWC_CntrGains_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%AWC_CntrGains(vit_CntrPar_AWC_CntrGains_n))
            IF (vit_CntrPar_AWC_CntrGains_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_AWC_CntrGains_ptr)) &
                vit_local_CntrPar%AWC_CntrGains(1:vit_CntrPar_AWC_CntrGains_n) = &
                    vit_tmp_CntrPar_AWC_CntrGains(1:vit_CntrPar_AWC_CntrGains_n)
        END IF
        vit_local_CntrPar%PF_Mode = vit_CntrPar_PF_Mode
        IF (C_ASSOCIATED(vit_CntrPar_PF_Offsets_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PF_Offsets_ptr, vit_tmp_CntrPar_PF_Offsets, [MAX(vit_CntrPar_PF_Offsets_cap, &
                1)])
        IF (vit_CntrPar_PF_Offsets_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PF_Offsets(vit_CntrPar_PF_Offsets_n))
            IF (vit_CntrPar_PF_Offsets_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PF_Offsets_ptr)) &
                vit_local_CntrPar%PF_Offsets(1:vit_CntrPar_PF_Offsets_n) = &
                    vit_tmp_CntrPar_PF_Offsets(1:vit_CntrPar_PF_Offsets_n)
        END IF
        IF (C_ASSOCIATED(vit_CntrPar_PF_TimeStuck_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_PF_TimeStuck_ptr, vit_tmp_CntrPar_PF_TimeStuck, &
                [MAX(vit_CntrPar_PF_TimeStuck_cap, 1)])
        IF (vit_CntrPar_PF_TimeStuck_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%PF_TimeStuck(vit_CntrPar_PF_TimeStuck_n))
            IF (vit_CntrPar_PF_TimeStuck_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_PF_TimeStuck_ptr)) &
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
        IF (C_ASSOCIATED(vit_CntrPar_CC_GroupIndex_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_CC_GroupIndex_ptr, vit_tmp_CntrPar_CC_GroupIndex, &
                [MAX(vit_CntrPar_CC_GroupIndex_cap, 1)])
        IF (vit_CntrPar_CC_GroupIndex_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%CC_GroupIndex(vit_CntrPar_CC_GroupIndex_n))
            IF (vit_CntrPar_CC_GroupIndex_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_CC_GroupIndex_ptr)) &
                vit_local_CntrPar%CC_GroupIndex(1:vit_CntrPar_CC_GroupIndex_n) = &
                    vit_tmp_CntrPar_CC_GroupIndex(1:vit_CntrPar_CC_GroupIndex_n)
        END IF
        vit_local_CntrPar%StC_Mode = vit_CntrPar_StC_Mode
        vit_local_CntrPar%StC_Group_N = vit_CntrPar_StC_Group_N
        IF (C_ASSOCIATED(vit_CntrPar_StC_GroupIndex_ptr)) &
            CALL C_F_POINTER(vit_CntrPar_StC_GroupIndex_ptr, vit_tmp_CntrPar_StC_GroupIndex, &
                [MAX(vit_CntrPar_StC_GroupIndex_cap, 1)])
        IF (vit_CntrPar_StC_GroupIndex_n >= 0) THEN
            ALLOCATE(vit_local_CntrPar%StC_GroupIndex(vit_CntrPar_StC_GroupIndex_n))
            IF (vit_CntrPar_StC_GroupIndex_n > 0 .AND. C_ASSOCIATED(vit_CntrPar_StC_GroupIndex_ptr)) &
                vit_local_CntrPar%StC_GroupIndex(1:vit_CntrPar_StC_GroupIndex_n) = &
                    vit_tmp_CntrPar_StC_GroupIndex(1:vit_CntrPar_StC_GroupIndex_n)
        END IF
        vit_local_CntrPar%PC_RtTq99 = vit_CntrPar_PC_RtTq99
        vit_local_CntrPar%VS_MaxOMTq = vit_CntrPar_VS_MaxOMTq
        vit_local_CntrPar%VS_MinOMTq = vit_CntrPar_VS_MinOMTq
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
        CALL UpdateZeroMQ(vit_local_LocalVar, vit_local_CntrPar, vit_local_ErrVar)
        vit_LocalVar_iStatus = vit_local_LocalVar%iStatus
        vit_LocalVar_AlreadyInitialized = vit_local_LocalVar%AlreadyInitialized
        vit_LocalVar_RestartWSE = vit_local_LocalVar%RestartWSE
        vit_LocalVar_Time = vit_local_LocalVar%Time
        vit_LocalVar_DT = vit_local_LocalVar%DT
        vit_LocalVar_WriteThisStep = vit_local_LocalVar%WriteThisStep
        vit_LocalVar_n_DT = vit_local_LocalVar%n_DT
        vit_LocalVar_Time_Last = vit_local_LocalVar%Time_Last
        vit_LocalVar_VS_GenPwr = vit_local_LocalVar%VS_GenPwr
        vit_LocalVar_GenSpeed = vit_local_LocalVar%GenSpeed
        vit_LocalVar_RotSpeed = vit_local_LocalVar%RotSpeed
        vit_LocalVar_NacHeading = vit_local_LocalVar%NacHeading
        vit_LocalVar_NacVane = vit_local_LocalVar%NacVane
        vit_LocalVar_NacVaneF = vit_local_LocalVar%NacVaneF
        vit_LocalVar_WindDir = vit_local_LocalVar%WindDir
        vit_LocalVar_HorWindV = vit_local_LocalVar%HorWindV
        vit_LocalVar_HorWindV_F = vit_local_LocalVar%HorWindV_F
        vit_LocalVar_rootMOOP = vit_local_LocalVar%rootMOOP
        vit_LocalVar_rootMOOPF = vit_local_LocalVar%rootMOOPF
        vit_LocalVar_BlPitch = vit_local_LocalVar%BlPitch
        vit_LocalVar_BlPitchCMeas = vit_local_LocalVar%BlPitchCMeas
        vit_LocalVar_Azimuth = vit_local_LocalVar%Azimuth
        vit_LocalVar_OL_Azimuth = vit_local_LocalVar%OL_Azimuth
        vit_LocalVar_AzUnwrapped = vit_local_LocalVar%AzUnwrapped
        vit_LocalVar_AzError = vit_local_LocalVar%AzError
        vit_LocalVar_GenTqAz = vit_local_LocalVar%GenTqAz
        vit_LocalVar_AzBuffer = vit_local_LocalVar%AzBuffer
        vit_LocalVar_NumBl = vit_local_LocalVar%NumBl
        vit_LocalVar_FA_Acc_TT = vit_local_LocalVar%FA_Acc_TT
        vit_LocalVar_SS_Acc_TT = vit_local_LocalVar%SS_Acc_TT
        vit_LocalVar_FA_Acc_Nac = vit_local_LocalVar%FA_Acc_Nac
        vit_LocalVar_NacIMU_FA_RAcc = vit_local_LocalVar%NacIMU_FA_RAcc
        vit_LocalVar_FA_AccHPF = vit_local_LocalVar%FA_AccHPF
        vit_LocalVar_FA_AccHPFI = vit_local_LocalVar%FA_AccHPFI
        vit_LocalVar_FA_PitCom = vit_local_LocalVar%FA_PitCom
        vit_LocalVar_VS_RefSpd = vit_local_LocalVar%VS_RefSpd
        vit_LocalVar_VS_RefSpd_TSR = vit_local_LocalVar%VS_RefSpd_TSR
        vit_LocalVar_VS_RefSpd_TRA = vit_local_LocalVar%VS_RefSpd_TRA
        vit_LocalVar_VS_RefSpd_RL = vit_local_LocalVar%VS_RefSpd_RL
        vit_LocalVar_PC_RefSpd = vit_local_LocalVar%PC_RefSpd
        vit_LocalVar_PC_RefSpd_SS = vit_local_LocalVar%PC_RefSpd_SS
        vit_LocalVar_PC_RefSpd_PRC = vit_local_LocalVar%PC_RefSpd_PRC
        vit_LocalVar_RotSpeedF = vit_local_LocalVar%RotSpeedF
        vit_LocalVar_GenSpeedF = vit_local_LocalVar%GenSpeedF
        vit_LocalVar_GenTq = vit_local_LocalVar%GenTq
        vit_LocalVar_GenTqMeas = vit_local_LocalVar%GenTqMeas
        vit_LocalVar_GenArTq = vit_local_LocalVar%GenArTq
        vit_LocalVar_GenBrTq = vit_local_LocalVar%GenBrTq
        vit_LocalVar_VS_KOmega2_GenTq = vit_local_LocalVar%VS_KOmega2_GenTq
        vit_LocalVar_VS_ConstPwr_GenTq = vit_local_LocalVar%VS_ConstPwr_GenTq
        vit_LocalVar_IPC_PitComF = vit_local_LocalVar%IPC_PitComF
        vit_LocalVar_PC_KP = vit_local_LocalVar%PC_KP
        vit_LocalVar_PC_KI = vit_local_LocalVar%PC_KI
        vit_LocalVar_PC_KD = vit_local_LocalVar%PC_KD
        vit_LocalVar_PC_TF = vit_local_LocalVar%PC_TF
        vit_LocalVar_PC_MaxPit = vit_local_LocalVar%PC_MaxPit
        vit_LocalVar_PC_MinPit = vit_local_LocalVar%PC_MinPit
        vit_LocalVar_PC_PitComT = vit_local_LocalVar%PC_PitComT
        vit_LocalVar_PC_PitComT_Last = vit_local_LocalVar%PC_PitComT_Last
        vit_LocalVar_BlPitchCMeasF = vit_local_LocalVar%BlPitchCMeasF
        vit_LocalVar_PC_PitComT_IPC = vit_local_LocalVar%PC_PitComT_IPC
        vit_LocalVar_PC_PwrErr = vit_local_LocalVar%PC_PwrErr
        vit_LocalVar_PC_SpdErr = vit_local_LocalVar%PC_SpdErr
        vit_LocalVar_IPC_AxisTilt_1P = vit_local_LocalVar%IPC_AxisTilt_1P
        vit_LocalVar_IPC_AxisYaw_1P = vit_local_LocalVar%IPC_AxisYaw_1P
        vit_LocalVar_IPC_AxisTilt_2P = vit_local_LocalVar%IPC_AxisTilt_2P
        vit_LocalVar_IPC_AxisYaw_2P = vit_local_LocalVar%IPC_AxisYaw_2P
        vit_LocalVar_axisTilt_1P = vit_local_LocalVar%axisTilt_1P
        vit_LocalVar_axisYaw_1P = vit_local_LocalVar%axisYaw_1P
        vit_LocalVar_axisYawF_1P = vit_local_LocalVar%axisYawF_1P
        vit_LocalVar_axisTilt_2P = vit_local_LocalVar%axisTilt_2P
        vit_LocalVar_axisYaw_2P = vit_local_LocalVar%axisYaw_2P
        vit_LocalVar_axisYawF_2P = vit_local_LocalVar%axisYawF_2P
        vit_LocalVar_IPC_KI = vit_local_LocalVar%IPC_KI
        vit_LocalVar_IPC_KP = vit_local_LocalVar%IPC_KP
        vit_LocalVar_IPC_IntSat = vit_local_LocalVar%IPC_IntSat
        vit_LocalVar_PC_State = vit_local_LocalVar%PC_State
        vit_LocalVar_PitCom = vit_local_LocalVar%PitCom
        vit_LocalVar_PitCom_SD = vit_local_LocalVar%PitCom_SD
        vit_LocalVar_PitComAct = vit_local_LocalVar%PitComAct
        vit_LocalVar_SS_DelOmegaF = vit_local_LocalVar%SS_DelOmegaF
        vit_LocalVar_TestType = vit_local_LocalVar%TestType
        vit_LocalVar_Kp_Float = vit_local_LocalVar%Kp_Float
        vit_LocalVar_VS_MaxTq = vit_local_LocalVar%VS_MaxTq
        vit_LocalVar_VS_LastGenTrq = vit_local_LocalVar%VS_LastGenTrq
        vit_LocalVar_VS_LastGenPwr = vit_local_LocalVar%VS_LastGenPwr
        vit_LocalVar_VS_MechGenPwr = vit_local_LocalVar%VS_MechGenPwr
        vit_LocalVar_VS_SpdErrAr = vit_local_LocalVar%VS_SpdErrAr
        vit_LocalVar_VS_SpdErrBr = vit_local_LocalVar%VS_SpdErrBr
        vit_LocalVar_VS_SpdErr = vit_local_LocalVar%VS_SpdErr
        vit_LocalVar_VS_State = vit_local_LocalVar%VS_State
        vit_LocalVar_VS_Rgn3Pitch = vit_local_LocalVar%VS_Rgn3Pitch
        vit_LocalVar_WE_Vw = vit_local_LocalVar%WE_Vw
        vit_LocalVar_WE_Vw_F = vit_local_LocalVar%WE_Vw_F
        vit_LocalVar_WE_VwI = vit_local_LocalVar%WE_VwI
        vit_LocalVar_WE_VwIdot = vit_local_LocalVar%WE_VwIdot
        vit_LocalVar_WE_Op = vit_local_LocalVar%WE_Op
        vit_LocalVar_WE_Op_Last = vit_local_LocalVar%WE_Op_Last
        vit_LocalVar_VS_LastGenTrqF = vit_local_LocalVar%VS_LastGenTrqF
        vit_LocalVar_PRC_WSE_F = vit_local_LocalVar%PRC_WSE_F
        vit_LocalVar_PRC_R_Speed = vit_local_LocalVar%PRC_R_Speed
        vit_LocalVar_PRC_R_Torque = vit_local_LocalVar%PRC_R_Torque
        vit_LocalVar_PRC_R_Pitch = vit_local_LocalVar%PRC_R_Pitch
        vit_LocalVar_PRC_R_Total = vit_local_LocalVar%PRC_R_Total
        vit_LocalVar_PRC_Min_Pitch = vit_local_LocalVar%PRC_Min_Pitch
        vit_LocalVar_PS_Min_Pitch = vit_local_LocalVar%PS_Min_Pitch
        vit_LocalVar_OL_Index = vit_local_LocalVar%OL_Index
        vit_LocalVar_SU_Stage = vit_local_LocalVar%SU_Stage
        vit_LocalVar_SU_LoadStageStartTime = vit_local_LocalVar%SU_LoadStageStartTime
        vit_LocalVar_SU_RotSpeedF = vit_local_LocalVar%SU_RotSpeedF
        vit_LocalVar_SD_Trigger = vit_local_LocalVar%SD_Trigger
        vit_LocalVar_SD_BlPitchF = vit_local_LocalVar%SD_BlPitchF
        vit_LocalVar_SD_NacVaneF = vit_local_LocalVar%SD_NacVaneF
        vit_LocalVar_SD_GenSpeedF = vit_local_LocalVar%SD_GenSpeedF
        vit_LocalVar_SD_Stage = vit_local_LocalVar%SD_Stage
        vit_LocalVar_SD_StageStartTime = vit_local_LocalVar%SD_StageStartTime
        vit_LocalVar_SD_MaxPitchRate = vit_local_LocalVar%SD_MaxPitchRate
        vit_LocalVar_SD_MaxTorqueRate = vit_local_LocalVar%SD_MaxTorqueRate
        vit_LocalVar_GenTq_SD = vit_local_LocalVar%GenTq_SD
        vit_LocalVar_Fl_PitCom = vit_local_LocalVar%Fl_PitCom
        vit_LocalVar_NACIMU_FA_AccF = vit_local_LocalVar%NACIMU_FA_AccF
        vit_LocalVar_FA_AccF = vit_local_LocalVar%FA_AccF
        vit_LocalVar_FA_Hist = vit_local_LocalVar%FA_Hist
        vit_LocalVar_TRA_LastRefSpd = vit_local_LocalVar%TRA_LastRefSpd
        vit_LocalVar_VS_RefSpeed = vit_local_LocalVar%VS_RefSpeed
        vit_LocalVar_PtfmTDX = vit_local_LocalVar%PtfmTDX
        vit_LocalVar_PtfmTDY = vit_local_LocalVar%PtfmTDY
        vit_LocalVar_PtfmTDZ = vit_local_LocalVar%PtfmTDZ
        vit_LocalVar_PtfmRDX = vit_local_LocalVar%PtfmRDX
        vit_LocalVar_PtfmRDY = vit_local_LocalVar%PtfmRDY
        vit_LocalVar_PtfmRDZ = vit_local_LocalVar%PtfmRDZ
        vit_LocalVar_PtfmTVX = vit_local_LocalVar%PtfmTVX
        vit_LocalVar_PtfmTVY = vit_local_LocalVar%PtfmTVY
        vit_LocalVar_PtfmTVZ = vit_local_LocalVar%PtfmTVZ
        vit_LocalVar_PtfmRVX = vit_local_LocalVar%PtfmRVX
        vit_LocalVar_PtfmRVY = vit_local_LocalVar%PtfmRVY
        vit_LocalVar_PtfmRVZ = vit_local_LocalVar%PtfmRVZ
        vit_LocalVar_PtfmTAX = vit_local_LocalVar%PtfmTAX
        vit_LocalVar_PtfmTAY = vit_local_LocalVar%PtfmTAY
        vit_LocalVar_PtfmTAZ = vit_local_LocalVar%PtfmTAZ
        vit_LocalVar_PtfmRAX = vit_local_LocalVar%PtfmRAX
        vit_LocalVar_PtfmRAY = vit_local_LocalVar%PtfmRAY
        vit_LocalVar_PtfmRAZ = vit_local_LocalVar%PtfmRAZ
        vit_LocalVar_CC_DesiredL = vit_local_LocalVar%CC_DesiredL
        vit_LocalVar_CC_ActuatedL = vit_local_LocalVar%CC_ActuatedL
        vit_LocalVar_CC_ActuatedDL = vit_local_LocalVar%CC_ActuatedDL
        vit_LocalVar_StC_Input = vit_local_LocalVar%StC_Input
        vit_LocalVar_Flp_Angle = vit_local_LocalVar%Flp_Angle
        vit_LocalVar_RootMyb_Last = vit_local_LocalVar%RootMyb_Last
        vit_LocalVar_ACC_INFILE_SIZE = vit_local_LocalVar%ACC_INFILE_SIZE
        IF (.NOT. ALLOCATED(vit_local_LocalVar%ACC_INFILE)) THEN
            vit_LocalVar_ACC_INFILE_n = -1_C_INT
        ELSE IF (SIZE(vit_local_LocalVar%ACC_INFILE) == 0) THEN
            vit_LocalVar_ACC_INFILE_n = 0_C_INT
        ELSE IF (SIZE(vit_local_LocalVar%ACC_INFILE) <= vit_LocalVar_ACC_INFILE_cap .AND. &
            C_ASSOCIATED(vit_LocalVar_ACC_INFILE_ptr)) THEN
            vit_LocalVar_ACC_INFILE_n = INT(SIZE(vit_local_LocalVar%ACC_INFILE), C_INT)
            IF (vit_LocalVar_ACC_INFILE_n > 0) &
                vit_tmp_LocalVar_ACC_INFILE(1:vit_LocalVar_ACC_INFILE_n) = &
                    vit_local_LocalVar%ACC_INFILE(1:vit_LocalVar_ACC_INFILE_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: LocalVar%ACC_INFILE came back with ', &
                SIZE(vit_local_LocalVar%ACC_INFILE), ' element(s), past the ', &
                vit_LocalVar_ACC_INFILE_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_LocalVar_restart = vit_local_LocalVar%restart
        vit_LocalVar_AWC_complexangle = vit_local_LocalVar%AWC_complexangle
        vit_LocalVar_TiltMean = vit_local_LocalVar%TiltMean
        vit_LocalVar_YawMean = vit_local_LocalVar%YawMean
        vit_LocalVar_ZMQ_ID = vit_local_LocalVar%ZMQ_ID
        vit_LocalVar_ZMQ_YawOffset = vit_local_LocalVar%ZMQ_YawOffset
        vit_LocalVar_ZMQ_TorqueOffset = vit_local_LocalVar%ZMQ_TorqueOffset
        vit_LocalVar_ZMQ_PitOffset = vit_local_LocalVar%ZMQ_PitOffset
        vit_LocalVar_ZMQ_R_Speed = vit_local_LocalVar%ZMQ_R_Speed
        vit_LocalVar_ZMQ_R_Torque = vit_local_LocalVar%ZMQ_R_Torque
        vit_LocalVar_ZMQ_R_Pitch = vit_local_LocalVar%ZMQ_R_Pitch
        IF (C_ASSOCIATED(vit_LocalVar_WE_ptr)) vit_nested_LocalVar_WE = vit_local_LocalVar%WE
        IF (C_ASSOCIATED(vit_LocalVar_FP_ptr)) vit_nested_LocalVar_FP = vit_local_LocalVar%FP
        IF (C_ASSOCIATED(vit_LocalVar_piP_ptr)) vit_nested_LocalVar_piP = vit_local_LocalVar%piP
        IF (C_ASSOCIATED(vit_LocalVar_resP_ptr)) vit_nested_LocalVar_resP = vit_local_LocalVar%resP
        IF (C_ASSOCIATED(vit_LocalVar_rlP_ptr)) vit_nested_LocalVar_rlP = vit_local_LocalVar%rlP
        IF (ALLOCATED(vit_local_LocalVar%ACC_INFILE)) DEALLOCATE(vit_local_LocalVar%ACC_INFILE)
        vit_CntrPar_ZMQ_ID = vit_local_CntrPar%ZMQ_ID
        vit_CntrPar_LoggingLevel = vit_local_CntrPar%LoggingLevel
        vit_CntrPar_Echo = vit_local_CntrPar%Echo
        vit_CntrPar_Ext_Interface = vit_local_CntrPar%Ext_Interface
        vit_CntrPar_DT_Out = vit_local_CntrPar%DT_Out
        vit_CntrPar_n_DT_Out = vit_local_CntrPar%n_DT_Out
        vit_CntrPar_n_DT_ZMQ = vit_local_CntrPar%n_DT_ZMQ
        vit_CntrPar_F_LPFType = vit_local_CntrPar%F_LPFType
        vit_CntrPar_F_LPFCornerFreq = vit_local_CntrPar%F_LPFCornerFreq
        vit_CntrPar_F_LPFDamping = vit_local_CntrPar%F_LPFDamping
        vit_CntrPar_F_NumNotchFilts = vit_local_CntrPar%F_NumNotchFilts
        vit_CntrPar_F_GenSpdNotch_N = vit_local_CntrPar%F_GenSpdNotch_N
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_GenSpdNotch_Ind)) THEN
            vit_CntrPar_F_GenSpdNotch_Ind_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_GenSpdNotch_Ind) == 0) THEN
            vit_CntrPar_F_GenSpdNotch_Ind_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_GenSpdNotch_Ind) <= vit_CntrPar_F_GenSpdNotch_Ind_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_GenSpdNotch_Ind_ptr)) THEN
            vit_CntrPar_F_GenSpdNotch_Ind_n = INT(SIZE(vit_local_CntrPar%F_GenSpdNotch_Ind), C_INT)
            IF (vit_CntrPar_F_GenSpdNotch_Ind_n > 0) &
                vit_tmp_CntrPar_F_GenSpdNotch_Ind(1:vit_CntrPar_F_GenSpdNotch_Ind_n) = &
                    vit_local_CntrPar%F_GenSpdNotch_Ind(1:vit_CntrPar_F_GenSpdNotch_Ind_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_GenSpdNotch_Ind came back with ', &
                SIZE(vit_local_CntrPar%F_GenSpdNotch_Ind), ' element(s), past the ', &
                vit_CntrPar_F_GenSpdNotch_Ind_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_F_TwrTopNotch_N = vit_local_CntrPar%F_TwrTopNotch_N
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_TwrTopNotch_Ind)) THEN
            vit_CntrPar_F_TwrTopNotch_Ind_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_TwrTopNotch_Ind) == 0) THEN
            vit_CntrPar_F_TwrTopNotch_Ind_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_TwrTopNotch_Ind) <= vit_CntrPar_F_TwrTopNotch_Ind_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_TwrTopNotch_Ind_ptr)) THEN
            vit_CntrPar_F_TwrTopNotch_Ind_n = INT(SIZE(vit_local_CntrPar%F_TwrTopNotch_Ind), C_INT)
            IF (vit_CntrPar_F_TwrTopNotch_Ind_n > 0) &
                vit_tmp_CntrPar_F_TwrTopNotch_Ind(1:vit_CntrPar_F_TwrTopNotch_Ind_n) = &
                    vit_local_CntrPar%F_TwrTopNotch_Ind(1:vit_CntrPar_F_TwrTopNotch_Ind_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_TwrTopNotch_Ind came back with ', &
                SIZE(vit_local_CntrPar%F_TwrTopNotch_Ind), ' element(s), past the ', &
                vit_CntrPar_F_TwrTopNotch_Ind_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_NotchFreqs)) THEN
            vit_CntrPar_F_NotchFreqs_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_NotchFreqs) == 0) THEN
            vit_CntrPar_F_NotchFreqs_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_NotchFreqs) <= vit_CntrPar_F_NotchFreqs_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_NotchFreqs_ptr)) THEN
            vit_CntrPar_F_NotchFreqs_n = INT(SIZE(vit_local_CntrPar%F_NotchFreqs), C_INT)
            IF (vit_CntrPar_F_NotchFreqs_n > 0) &
                vit_tmp_CntrPar_F_NotchFreqs(1:vit_CntrPar_F_NotchFreqs_n) = &
                    vit_local_CntrPar%F_NotchFreqs(1:vit_CntrPar_F_NotchFreqs_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_NotchFreqs came back with ', &
                SIZE(vit_local_CntrPar%F_NotchFreqs), ' element(s), past the ', &
                vit_CntrPar_F_NotchFreqs_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_NotchBetaNum)) THEN
            vit_CntrPar_F_NotchBetaNum_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_NotchBetaNum) == 0) THEN
            vit_CntrPar_F_NotchBetaNum_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_NotchBetaNum) <= vit_CntrPar_F_NotchBetaNum_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_NotchBetaNum_ptr)) THEN
            vit_CntrPar_F_NotchBetaNum_n = INT(SIZE(vit_local_CntrPar%F_NotchBetaNum), C_INT)
            IF (vit_CntrPar_F_NotchBetaNum_n > 0) &
                vit_tmp_CntrPar_F_NotchBetaNum(1:vit_CntrPar_F_NotchBetaNum_n) = &
                    vit_local_CntrPar%F_NotchBetaNum(1:vit_CntrPar_F_NotchBetaNum_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_NotchBetaNum came back with ', &
                SIZE(vit_local_CntrPar%F_NotchBetaNum), ' element(s), past the ', &
                vit_CntrPar_F_NotchBetaNum_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_NotchBetaDen)) THEN
            vit_CntrPar_F_NotchBetaDen_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_NotchBetaDen) == 0) THEN
            vit_CntrPar_F_NotchBetaDen_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_NotchBetaDen) <= vit_CntrPar_F_NotchBetaDen_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_NotchBetaDen_ptr)) THEN
            vit_CntrPar_F_NotchBetaDen_n = INT(SIZE(vit_local_CntrPar%F_NotchBetaDen), C_INT)
            IF (vit_CntrPar_F_NotchBetaDen_n > 0) &
                vit_tmp_CntrPar_F_NotchBetaDen(1:vit_CntrPar_F_NotchBetaDen_n) = &
                    vit_local_CntrPar%F_NotchBetaDen(1:vit_CntrPar_F_NotchBetaDen_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_NotchBetaDen came back with ', &
                SIZE(vit_local_CntrPar%F_NotchBetaDen), ' element(s), past the ', &
                vit_CntrPar_F_NotchBetaDen_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_F_SSCornerFreq = vit_local_CntrPar%F_SSCornerFreq
        vit_CntrPar_F_WECornerFreq = vit_local_CntrPar%F_WECornerFreq
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_FlCornerFreq)) THEN
            vit_CntrPar_F_FlCornerFreq_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_FlCornerFreq) == 0) THEN
            vit_CntrPar_F_FlCornerFreq_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_FlCornerFreq) <= vit_CntrPar_F_FlCornerFreq_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_FlCornerFreq_ptr)) THEN
            vit_CntrPar_F_FlCornerFreq_n = INT(SIZE(vit_local_CntrPar%F_FlCornerFreq), C_INT)
            IF (vit_CntrPar_F_FlCornerFreq_n > 0) &
                vit_tmp_CntrPar_F_FlCornerFreq(1:vit_CntrPar_F_FlCornerFreq_n) = &
                    vit_local_CntrPar%F_FlCornerFreq(1:vit_CntrPar_F_FlCornerFreq_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_FlCornerFreq came back with ', &
                SIZE(vit_local_CntrPar%F_FlCornerFreq), ' element(s), past the ', &
                vit_CntrPar_F_FlCornerFreq_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_F_FlHighPassFreq = vit_local_CntrPar%F_FlHighPassFreq
        vit_CntrPar_F_YawErr = vit_local_CntrPar%F_YawErr
        IF (.NOT. ALLOCATED(vit_local_CntrPar%F_FlpCornerFreq)) THEN
            vit_CntrPar_F_FlpCornerFreq_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_FlpCornerFreq) == 0) THEN
            vit_CntrPar_F_FlpCornerFreq_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%F_FlpCornerFreq) <= vit_CntrPar_F_FlpCornerFreq_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_F_FlpCornerFreq_ptr)) THEN
            vit_CntrPar_F_FlpCornerFreq_n = INT(SIZE(vit_local_CntrPar%F_FlpCornerFreq), C_INT)
            IF (vit_CntrPar_F_FlpCornerFreq_n > 0) &
                vit_tmp_CntrPar_F_FlpCornerFreq(1:vit_CntrPar_F_FlpCornerFreq_n) = &
                    vit_local_CntrPar%F_FlpCornerFreq(1:vit_CntrPar_F_FlpCornerFreq_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%F_FlpCornerFreq came back with ', &
                SIZE(vit_local_CntrPar%F_FlpCornerFreq), ' element(s), past the ', &
                vit_CntrPar_F_FlpCornerFreq_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_F_VSRefSpdCornerFreq = vit_local_CntrPar%F_VSRefSpdCornerFreq
        vit_CntrPar_TRA_Mode = vit_local_CntrPar%TRA_Mode
        vit_CntrPar_TRA_ExclSpeed = vit_local_CntrPar%TRA_ExclSpeed
        vit_CntrPar_TRA_ExclBand = vit_local_CntrPar%TRA_ExclBand
        vit_CntrPar_TRA_RateLimit = vit_local_CntrPar%TRA_RateLimit
        vit_CntrPar_TD_Mode = vit_local_CntrPar%TD_Mode
        vit_CntrPar_FA_HPFCornerFreq = vit_local_CntrPar%FA_HPFCornerFreq
        vit_CntrPar_FA_IntSat = vit_local_CntrPar%FA_IntSat
        vit_CntrPar_FA_KI = vit_local_CntrPar%FA_KI
        vit_CntrPar_IPC_ControlMode = vit_local_CntrPar%IPC_ControlMode
        IF (.NOT. ALLOCATED(vit_local_CntrPar%IPC_Vramp)) THEN
            vit_CntrPar_IPC_Vramp_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_Vramp) == 0) THEN
            vit_CntrPar_IPC_Vramp_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_Vramp) <= vit_CntrPar_IPC_Vramp_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_IPC_Vramp_ptr)) THEN
            vit_CntrPar_IPC_Vramp_n = INT(SIZE(vit_local_CntrPar%IPC_Vramp), C_INT)
            IF (vit_CntrPar_IPC_Vramp_n > 0) &
                vit_tmp_CntrPar_IPC_Vramp(1:vit_CntrPar_IPC_Vramp_n) = &
                    vit_local_CntrPar%IPC_Vramp(1:vit_CntrPar_IPC_Vramp_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%IPC_Vramp came back with ', &
                SIZE(vit_local_CntrPar%IPC_Vramp), ' element(s), past the ', &
                vit_CntrPar_IPC_Vramp_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_IPC_IntSat = vit_local_CntrPar%IPC_IntSat
        vit_CntrPar_IPC_SatMode = vit_local_CntrPar%IPC_SatMode
        IF (.NOT. ALLOCATED(vit_local_CntrPar%IPC_KP)) THEN
            vit_CntrPar_IPC_KP_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_KP) == 0) THEN
            vit_CntrPar_IPC_KP_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_KP) <= vit_CntrPar_IPC_KP_cap .AND. C_ASSOCIATED(vit_CntrPar_IPC_KP_ptr)) &
            THEN
            vit_CntrPar_IPC_KP_n = INT(SIZE(vit_local_CntrPar%IPC_KP), C_INT)
            IF (vit_CntrPar_IPC_KP_n > 0) &
                vit_tmp_CntrPar_IPC_KP(1:vit_CntrPar_IPC_KP_n) = vit_local_CntrPar%IPC_KP(1:vit_CntrPar_IPC_KP_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%IPC_KP came back with ', &
                SIZE(vit_local_CntrPar%IPC_KP), ' element(s), past the ', &
                vit_CntrPar_IPC_KP_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%IPC_KI)) THEN
            vit_CntrPar_IPC_KI_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_KI) == 0) THEN
            vit_CntrPar_IPC_KI_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_KI) <= vit_CntrPar_IPC_KI_cap .AND. C_ASSOCIATED(vit_CntrPar_IPC_KI_ptr)) &
            THEN
            vit_CntrPar_IPC_KI_n = INT(SIZE(vit_local_CntrPar%IPC_KI), C_INT)
            IF (vit_CntrPar_IPC_KI_n > 0) &
                vit_tmp_CntrPar_IPC_KI(1:vit_CntrPar_IPC_KI_n) = vit_local_CntrPar%IPC_KI(1:vit_CntrPar_IPC_KI_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%IPC_KI came back with ', &
                SIZE(vit_local_CntrPar%IPC_KI), ' element(s), past the ', &
                vit_CntrPar_IPC_KI_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%IPC_aziOffset)) THEN
            vit_CntrPar_IPC_aziOffset_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_aziOffset) == 0) THEN
            vit_CntrPar_IPC_aziOffset_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%IPC_aziOffset) <= vit_CntrPar_IPC_aziOffset_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_IPC_aziOffset_ptr)) THEN
            vit_CntrPar_IPC_aziOffset_n = INT(SIZE(vit_local_CntrPar%IPC_aziOffset), C_INT)
            IF (vit_CntrPar_IPC_aziOffset_n > 0) &
                vit_tmp_CntrPar_IPC_aziOffset(1:vit_CntrPar_IPC_aziOffset_n) = &
                    vit_local_CntrPar%IPC_aziOffset(1:vit_CntrPar_IPC_aziOffset_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%IPC_aziOffset came back with ', &
                SIZE(vit_local_CntrPar%IPC_aziOffset), ' element(s), past the ', &
                vit_CntrPar_IPC_aziOffset_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_IPC_CornerFreqAct = vit_local_CntrPar%IPC_CornerFreqAct
        vit_CntrPar_PC_ControlMode = vit_local_CntrPar%PC_ControlMode
        vit_CntrPar_PC_GS_n = vit_local_CntrPar%PC_GS_n
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PC_GS_angles)) THEN
            vit_CntrPar_PC_GS_angles_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_angles) == 0) THEN
            vit_CntrPar_PC_GS_angles_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_angles) <= vit_CntrPar_PC_GS_angles_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PC_GS_angles_ptr)) THEN
            vit_CntrPar_PC_GS_angles_n = INT(SIZE(vit_local_CntrPar%PC_GS_angles), C_INT)
            IF (vit_CntrPar_PC_GS_angles_n > 0) &
                vit_tmp_CntrPar_PC_GS_angles(1:vit_CntrPar_PC_GS_angles_n) = &
                    vit_local_CntrPar%PC_GS_angles(1:vit_CntrPar_PC_GS_angles_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PC_GS_angles came back with ', &
                SIZE(vit_local_CntrPar%PC_GS_angles), ' element(s), past the ', &
                vit_CntrPar_PC_GS_angles_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PC_GS_KP)) THEN
            vit_CntrPar_PC_GS_KP_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_KP) == 0) THEN
            vit_CntrPar_PC_GS_KP_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_KP) <= vit_CntrPar_PC_GS_KP_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PC_GS_KP_ptr)) THEN
            vit_CntrPar_PC_GS_KP_n = INT(SIZE(vit_local_CntrPar%PC_GS_KP), C_INT)
            IF (vit_CntrPar_PC_GS_KP_n > 0) &
                vit_tmp_CntrPar_PC_GS_KP(1:vit_CntrPar_PC_GS_KP_n) = &
                    vit_local_CntrPar%PC_GS_KP(1:vit_CntrPar_PC_GS_KP_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PC_GS_KP came back with ', &
                SIZE(vit_local_CntrPar%PC_GS_KP), ' element(s), past the ', &
                vit_CntrPar_PC_GS_KP_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PC_GS_KI)) THEN
            vit_CntrPar_PC_GS_KI_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_KI) == 0) THEN
            vit_CntrPar_PC_GS_KI_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_KI) <= vit_CntrPar_PC_GS_KI_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PC_GS_KI_ptr)) THEN
            vit_CntrPar_PC_GS_KI_n = INT(SIZE(vit_local_CntrPar%PC_GS_KI), C_INT)
            IF (vit_CntrPar_PC_GS_KI_n > 0) &
                vit_tmp_CntrPar_PC_GS_KI(1:vit_CntrPar_PC_GS_KI_n) = &
                    vit_local_CntrPar%PC_GS_KI(1:vit_CntrPar_PC_GS_KI_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PC_GS_KI came back with ', &
                SIZE(vit_local_CntrPar%PC_GS_KI), ' element(s), past the ', &
                vit_CntrPar_PC_GS_KI_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PC_GS_KD)) THEN
            vit_CntrPar_PC_GS_KD_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_KD) == 0) THEN
            vit_CntrPar_PC_GS_KD_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_KD) <= vit_CntrPar_PC_GS_KD_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PC_GS_KD_ptr)) THEN
            vit_CntrPar_PC_GS_KD_n = INT(SIZE(vit_local_CntrPar%PC_GS_KD), C_INT)
            IF (vit_CntrPar_PC_GS_KD_n > 0) &
                vit_tmp_CntrPar_PC_GS_KD(1:vit_CntrPar_PC_GS_KD_n) = &
                    vit_local_CntrPar%PC_GS_KD(1:vit_CntrPar_PC_GS_KD_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PC_GS_KD came back with ', &
                SIZE(vit_local_CntrPar%PC_GS_KD), ' element(s), past the ', &
                vit_CntrPar_PC_GS_KD_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PC_GS_TF)) THEN
            vit_CntrPar_PC_GS_TF_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_TF) == 0) THEN
            vit_CntrPar_PC_GS_TF_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PC_GS_TF) <= vit_CntrPar_PC_GS_TF_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PC_GS_TF_ptr)) THEN
            vit_CntrPar_PC_GS_TF_n = INT(SIZE(vit_local_CntrPar%PC_GS_TF), C_INT)
            IF (vit_CntrPar_PC_GS_TF_n > 0) &
                vit_tmp_CntrPar_PC_GS_TF(1:vit_CntrPar_PC_GS_TF_n) = &
                    vit_local_CntrPar%PC_GS_TF(1:vit_CntrPar_PC_GS_TF_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PC_GS_TF came back with ', &
                SIZE(vit_local_CntrPar%PC_GS_TF), ' element(s), past the ', &
                vit_CntrPar_PC_GS_TF_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_PC_MaxPit = vit_local_CntrPar%PC_MaxPit
        vit_CntrPar_PC_MinPit = vit_local_CntrPar%PC_MinPit
        vit_CntrPar_PC_MaxRat = vit_local_CntrPar%PC_MaxRat
        vit_CntrPar_PC_MinRat = vit_local_CntrPar%PC_MinRat
        vit_CntrPar_PC_RefSpd = vit_local_CntrPar%PC_RefSpd
        vit_CntrPar_PC_FinePit = vit_local_CntrPar%PC_FinePit
        vit_CntrPar_PC_Switch = vit_local_CntrPar%PC_Switch
        vit_CntrPar_VS_ControlMode = vit_local_CntrPar%VS_ControlMode
        vit_CntrPar_VS_ConstPower = vit_local_CntrPar%VS_ConstPower
        vit_CntrPar_VS_FBP = vit_local_CntrPar%VS_FBP
        vit_CntrPar_VS_GenEff = vit_local_CntrPar%VS_GenEff
        vit_CntrPar_VS_ArSatTq = vit_local_CntrPar%VS_ArSatTq
        vit_CntrPar_VS_MaxRat = vit_local_CntrPar%VS_MaxRat
        vit_CntrPar_VS_MaxTq = vit_local_CntrPar%VS_MaxTq
        vit_CntrPar_VS_MinTq = vit_local_CntrPar%VS_MinTq
        vit_CntrPar_VS_MinOMSpd = vit_local_CntrPar%VS_MinOMSpd
        vit_CntrPar_VS_Rgn2K = vit_local_CntrPar%VS_Rgn2K
        vit_CntrPar_VS_RtPwr = vit_local_CntrPar%VS_RtPwr
        vit_CntrPar_VS_RtTq = vit_local_CntrPar%VS_RtTq
        vit_CntrPar_VS_RefSpd = vit_local_CntrPar%VS_RefSpd
        vit_CntrPar_VS_n = vit_local_CntrPar%VS_n
        IF (.NOT. ALLOCATED(vit_local_CntrPar%VS_KP)) THEN
            vit_CntrPar_VS_KP_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_KP) == 0) THEN
            vit_CntrPar_VS_KP_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_KP) <= vit_CntrPar_VS_KP_cap .AND. C_ASSOCIATED(vit_CntrPar_VS_KP_ptr)) THEN
            vit_CntrPar_VS_KP_n = INT(SIZE(vit_local_CntrPar%VS_KP), C_INT)
            IF (vit_CntrPar_VS_KP_n > 0) &
                vit_tmp_CntrPar_VS_KP(1:vit_CntrPar_VS_KP_n) = vit_local_CntrPar%VS_KP(1:vit_CntrPar_VS_KP_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%VS_KP came back with ', &
                SIZE(vit_local_CntrPar%VS_KP), ' element(s), past the ', &
                vit_CntrPar_VS_KP_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%VS_KI)) THEN
            vit_CntrPar_VS_KI_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_KI) == 0) THEN
            vit_CntrPar_VS_KI_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_KI) <= vit_CntrPar_VS_KI_cap .AND. C_ASSOCIATED(vit_CntrPar_VS_KI_ptr)) THEN
            vit_CntrPar_VS_KI_n = INT(SIZE(vit_local_CntrPar%VS_KI), C_INT)
            IF (vit_CntrPar_VS_KI_n > 0) &
                vit_tmp_CntrPar_VS_KI(1:vit_CntrPar_VS_KI_n) = vit_local_CntrPar%VS_KI(1:vit_CntrPar_VS_KI_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%VS_KI came back with ', &
                SIZE(vit_local_CntrPar%VS_KI), ' element(s), past the ', &
                vit_CntrPar_VS_KI_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_VS_TSRopt = vit_local_CntrPar%VS_TSRopt
        vit_CntrPar_VS_FBP_n = vit_local_CntrPar%VS_FBP_n
        IF (.NOT. ALLOCATED(vit_local_CntrPar%VS_FBP_U)) THEN
            vit_CntrPar_VS_FBP_U_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_FBP_U) == 0) THEN
            vit_CntrPar_VS_FBP_U_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_FBP_U) <= vit_CntrPar_VS_FBP_U_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_VS_FBP_U_ptr)) THEN
            vit_CntrPar_VS_FBP_U_n = INT(SIZE(vit_local_CntrPar%VS_FBP_U), C_INT)
            IF (vit_CntrPar_VS_FBP_U_n > 0) &
                vit_tmp_CntrPar_VS_FBP_U(1:vit_CntrPar_VS_FBP_U_n) = &
                    vit_local_CntrPar%VS_FBP_U(1:vit_CntrPar_VS_FBP_U_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%VS_FBP_U came back with ', &
                SIZE(vit_local_CntrPar%VS_FBP_U), ' element(s), past the ', &
                vit_CntrPar_VS_FBP_U_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%VS_FBP_Omega)) THEN
            vit_CntrPar_VS_FBP_Omega_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_FBP_Omega) == 0) THEN
            vit_CntrPar_VS_FBP_Omega_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_FBP_Omega) <= vit_CntrPar_VS_FBP_Omega_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_VS_FBP_Omega_ptr)) THEN
            vit_CntrPar_VS_FBP_Omega_n = INT(SIZE(vit_local_CntrPar%VS_FBP_Omega), C_INT)
            IF (vit_CntrPar_VS_FBP_Omega_n > 0) &
                vit_tmp_CntrPar_VS_FBP_Omega(1:vit_CntrPar_VS_FBP_Omega_n) = &
                    vit_local_CntrPar%VS_FBP_Omega(1:vit_CntrPar_VS_FBP_Omega_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%VS_FBP_Omega came back with ', &
                SIZE(vit_local_CntrPar%VS_FBP_Omega), ' element(s), past the ', &
                vit_CntrPar_VS_FBP_Omega_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%VS_FBP_Tau)) THEN
            vit_CntrPar_VS_FBP_Tau_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_FBP_Tau) == 0) THEN
            vit_CntrPar_VS_FBP_Tau_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%VS_FBP_Tau) <= vit_CntrPar_VS_FBP_Tau_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_VS_FBP_Tau_ptr)) THEN
            vit_CntrPar_VS_FBP_Tau_n = INT(SIZE(vit_local_CntrPar%VS_FBP_Tau), C_INT)
            IF (vit_CntrPar_VS_FBP_Tau_n > 0) &
                vit_tmp_CntrPar_VS_FBP_Tau(1:vit_CntrPar_VS_FBP_Tau_n) = &
                    vit_local_CntrPar%VS_FBP_Tau(1:vit_CntrPar_VS_FBP_Tau_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%VS_FBP_Tau came back with ', &
                SIZE(vit_local_CntrPar%VS_FBP_Tau), ' element(s), past the ', &
                vit_CntrPar_VS_FBP_Tau_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_SS_Mode = vit_local_CntrPar%SS_Mode
        vit_CntrPar_SS_VSGain = vit_local_CntrPar%SS_VSGain
        vit_CntrPar_SS_PCGain = vit_local_CntrPar%SS_PCGain
        vit_CntrPar_PRC_Mode = vit_local_CntrPar%PRC_Mode
        vit_CntrPar_PRC_Comm = vit_local_CntrPar%PRC_Comm
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PRC_WindSpeeds)) THEN
            vit_CntrPar_PRC_WindSpeeds_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_WindSpeeds) == 0) THEN
            vit_CntrPar_PRC_WindSpeeds_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_WindSpeeds) <= vit_CntrPar_PRC_WindSpeeds_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PRC_WindSpeeds_ptr)) THEN
            vit_CntrPar_PRC_WindSpeeds_n = INT(SIZE(vit_local_CntrPar%PRC_WindSpeeds), C_INT)
            IF (vit_CntrPar_PRC_WindSpeeds_n > 0) &
                vit_tmp_CntrPar_PRC_WindSpeeds(1:vit_CntrPar_PRC_WindSpeeds_n) = &
                    vit_local_CntrPar%PRC_WindSpeeds(1:vit_CntrPar_PRC_WindSpeeds_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PRC_WindSpeeds came back with ', &
                SIZE(vit_local_CntrPar%PRC_WindSpeeds), ' element(s), past the ', &
                vit_CntrPar_PRC_WindSpeeds_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PRC_GenSpeeds)) THEN
            vit_CntrPar_PRC_GenSpeeds_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_GenSpeeds) == 0) THEN
            vit_CntrPar_PRC_GenSpeeds_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_GenSpeeds) <= vit_CntrPar_PRC_GenSpeeds_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PRC_GenSpeeds_ptr)) THEN
            vit_CntrPar_PRC_GenSpeeds_n = INT(SIZE(vit_local_CntrPar%PRC_GenSpeeds), C_INT)
            IF (vit_CntrPar_PRC_GenSpeeds_n > 0) &
                vit_tmp_CntrPar_PRC_GenSpeeds(1:vit_CntrPar_PRC_GenSpeeds_n) = &
                    vit_local_CntrPar%PRC_GenSpeeds(1:vit_CntrPar_PRC_GenSpeeds_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PRC_GenSpeeds came back with ', &
                SIZE(vit_local_CntrPar%PRC_GenSpeeds), ' element(s), past the ', &
                vit_CntrPar_PRC_GenSpeeds_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_PRC_n = vit_local_CntrPar%PRC_n
        vit_CntrPar_PRC_LPF_Freq = vit_local_CntrPar%PRC_LPF_Freq
        vit_CntrPar_PRC_R_Torque = vit_local_CntrPar%PRC_R_Torque
        vit_CntrPar_PRC_R_Speed = vit_local_CntrPar%PRC_R_Speed
        vit_CntrPar_PRC_R_Pitch = vit_local_CntrPar%PRC_R_Pitch
        vit_CntrPar_PRC_Table_n = vit_local_CntrPar%PRC_Table_n
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PRC_Pitch_Table)) THEN
            vit_CntrPar_PRC_Pitch_Table_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_Pitch_Table) == 0) THEN
            vit_CntrPar_PRC_Pitch_Table_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_Pitch_Table) <= vit_CntrPar_PRC_Pitch_Table_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PRC_Pitch_Table_ptr)) THEN
            vit_CntrPar_PRC_Pitch_Table_n = INT(SIZE(vit_local_CntrPar%PRC_Pitch_Table), C_INT)
            IF (vit_CntrPar_PRC_Pitch_Table_n > 0) &
                vit_tmp_CntrPar_PRC_Pitch_Table(1:vit_CntrPar_PRC_Pitch_Table_n) = &
                    vit_local_CntrPar%PRC_Pitch_Table(1:vit_CntrPar_PRC_Pitch_Table_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PRC_Pitch_Table came back with ', &
                SIZE(vit_local_CntrPar%PRC_Pitch_Table), ' element(s), past the ', &
                vit_CntrPar_PRC_Pitch_Table_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PRC_R_Table)) THEN
            vit_CntrPar_PRC_R_Table_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_R_Table) == 0) THEN
            vit_CntrPar_PRC_R_Table_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PRC_R_Table) <= vit_CntrPar_PRC_R_Table_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PRC_R_Table_ptr)) THEN
            vit_CntrPar_PRC_R_Table_n = INT(SIZE(vit_local_CntrPar%PRC_R_Table), C_INT)
            IF (vit_CntrPar_PRC_R_Table_n > 0) &
                vit_tmp_CntrPar_PRC_R_Table(1:vit_CntrPar_PRC_R_Table_n) = &
                    vit_local_CntrPar%PRC_R_Table(1:vit_CntrPar_PRC_R_Table_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PRC_R_Table came back with ', &
                SIZE(vit_local_CntrPar%PRC_R_Table), ' element(s), past the ', &
                vit_CntrPar_PRC_R_Table_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_WE_Mode = vit_local_CntrPar%WE_Mode
        vit_CntrPar_WE_BladeRadius = vit_local_CntrPar%WE_BladeRadius
        vit_CntrPar_WE_CP_n = vit_local_CntrPar%WE_CP_n
        IF (.NOT. ALLOCATED(vit_local_CntrPar%WE_CP)) THEN
            vit_CntrPar_WE_CP_n_v2 = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%WE_CP) == 0) THEN
            vit_CntrPar_WE_CP_n_v2 = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%WE_CP) <= vit_CntrPar_WE_CP_cap .AND. C_ASSOCIATED(vit_CntrPar_WE_CP_ptr)) THEN
            vit_CntrPar_WE_CP_n_v2 = INT(SIZE(vit_local_CntrPar%WE_CP), C_INT)
            IF (vit_CntrPar_WE_CP_n_v2 > 0) &
                vit_tmp_CntrPar_WE_CP(1:vit_CntrPar_WE_CP_n_v2) = vit_local_CntrPar%WE_CP(1:vit_CntrPar_WE_CP_n_v2)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%WE_CP came back with ', &
                SIZE(vit_local_CntrPar%WE_CP), ' element(s), past the ', &
                vit_CntrPar_WE_CP_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_WE_Gamma = vit_local_CntrPar%WE_Gamma
        vit_CntrPar_WE_GearboxRatio = vit_local_CntrPar%WE_GearboxRatio
        vit_CntrPar_WE_Jtot = vit_local_CntrPar%WE_Jtot
        vit_CntrPar_WE_RhoAir = vit_local_CntrPar%WE_RhoAir
        DO vit_ci_CntrPar_PerfFileName = 1, 1024
            vit_CntrPar_PerfFileName(vit_ci_CntrPar_PerfFileName) = &
                vit_local_CntrPar%PerfFileName(vit_ci_CntrPar_PerfFileName:vit_ci_CntrPar_PerfFileName)
        END DO
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PerfTableSize)) THEN
            vit_CntrPar_PerfTableSize_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PerfTableSize) == 0) THEN
            vit_CntrPar_PerfTableSize_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PerfTableSize) <= vit_CntrPar_PerfTableSize_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PerfTableSize_ptr)) THEN
            vit_CntrPar_PerfTableSize_n = INT(SIZE(vit_local_CntrPar%PerfTableSize), C_INT)
            IF (vit_CntrPar_PerfTableSize_n > 0) &
                vit_tmp_CntrPar_PerfTableSize(1:vit_CntrPar_PerfTableSize_n) = &
                    vit_local_CntrPar%PerfTableSize(1:vit_CntrPar_PerfTableSize_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PerfTableSize came back with ', &
                SIZE(vit_local_CntrPar%PerfTableSize), ' element(s), past the ', &
                vit_CntrPar_PerfTableSize_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_WE_FOPoles_N = vit_local_CntrPar%WE_FOPoles_N
        IF (.NOT. ALLOCATED(vit_local_CntrPar%WE_FOPoles_v)) THEN
            vit_CntrPar_WE_FOPoles_v_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%WE_FOPoles_v) == 0) THEN
            vit_CntrPar_WE_FOPoles_v_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%WE_FOPoles_v) <= vit_CntrPar_WE_FOPoles_v_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_WE_FOPoles_v_ptr)) THEN
            vit_CntrPar_WE_FOPoles_v_n = INT(SIZE(vit_local_CntrPar%WE_FOPoles_v), C_INT)
            IF (vit_CntrPar_WE_FOPoles_v_n > 0) &
                vit_tmp_CntrPar_WE_FOPoles_v(1:vit_CntrPar_WE_FOPoles_v_n) = &
                    vit_local_CntrPar%WE_FOPoles_v(1:vit_CntrPar_WE_FOPoles_v_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%WE_FOPoles_v came back with ', &
                SIZE(vit_local_CntrPar%WE_FOPoles_v), ' element(s), past the ', &
                vit_CntrPar_WE_FOPoles_v_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%WE_FOPoles)) THEN
            vit_CntrPar_WE_FOPoles_n_v2 = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%WE_FOPoles) == 0) THEN
            vit_CntrPar_WE_FOPoles_n_v2 = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%WE_FOPoles) <= vit_CntrPar_WE_FOPoles_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_WE_FOPoles_ptr)) THEN
            vit_CntrPar_WE_FOPoles_n_v2 = INT(SIZE(vit_local_CntrPar%WE_FOPoles), C_INT)
            IF (vit_CntrPar_WE_FOPoles_n_v2 > 0) &
                vit_tmp_CntrPar_WE_FOPoles(1:vit_CntrPar_WE_FOPoles_n_v2) = &
                    vit_local_CntrPar%WE_FOPoles(1:vit_CntrPar_WE_FOPoles_n_v2)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%WE_FOPoles came back with ', &
                SIZE(vit_local_CntrPar%WE_FOPoles), ' element(s), past the ', &
                vit_CntrPar_WE_FOPoles_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_Y_ControlMode = vit_local_CntrPar%Y_ControlMode
        vit_CntrPar_Y_uSwitch = vit_local_CntrPar%Y_uSwitch
        IF (.NOT. ALLOCATED(vit_local_CntrPar%Y_ErrThresh)) THEN
            vit_CntrPar_Y_ErrThresh_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Y_ErrThresh) == 0) THEN
            vit_CntrPar_Y_ErrThresh_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Y_ErrThresh) <= vit_CntrPar_Y_ErrThresh_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_Y_ErrThresh_ptr)) THEN
            vit_CntrPar_Y_ErrThresh_n = INT(SIZE(vit_local_CntrPar%Y_ErrThresh), C_INT)
            IF (vit_CntrPar_Y_ErrThresh_n > 0) &
                vit_tmp_CntrPar_Y_ErrThresh(1:vit_CntrPar_Y_ErrThresh_n) = &
                    vit_local_CntrPar%Y_ErrThresh(1:vit_CntrPar_Y_ErrThresh_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%Y_ErrThresh came back with ', &
                SIZE(vit_local_CntrPar%Y_ErrThresh), ' element(s), past the ', &
                vit_CntrPar_Y_ErrThresh_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_Y_Rate = vit_local_CntrPar%Y_Rate
        vit_CntrPar_Y_MErrSet = vit_local_CntrPar%Y_MErrSet
        vit_CntrPar_Y_IPC_IntSat = vit_local_CntrPar%Y_IPC_IntSat
        vit_CntrPar_Y_IPC_KP = vit_local_CntrPar%Y_IPC_KP
        vit_CntrPar_Y_IPC_KI = vit_local_CntrPar%Y_IPC_KI
        vit_CntrPar_PS_Mode = vit_local_CntrPar%PS_Mode
        vit_CntrPar_PS_BldPitchMin_N = vit_local_CntrPar%PS_BldPitchMin_N
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PS_WindSpeeds)) THEN
            vit_CntrPar_PS_WindSpeeds_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PS_WindSpeeds) == 0) THEN
            vit_CntrPar_PS_WindSpeeds_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PS_WindSpeeds) <= vit_CntrPar_PS_WindSpeeds_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PS_WindSpeeds_ptr)) THEN
            vit_CntrPar_PS_WindSpeeds_n = INT(SIZE(vit_local_CntrPar%PS_WindSpeeds), C_INT)
            IF (vit_CntrPar_PS_WindSpeeds_n > 0) &
                vit_tmp_CntrPar_PS_WindSpeeds(1:vit_CntrPar_PS_WindSpeeds_n) = &
                    vit_local_CntrPar%PS_WindSpeeds(1:vit_CntrPar_PS_WindSpeeds_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PS_WindSpeeds came back with ', &
                SIZE(vit_local_CntrPar%PS_WindSpeeds), ' element(s), past the ', &
                vit_CntrPar_PS_WindSpeeds_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PS_BldPitchMin)) THEN
            vit_CntrPar_PS_BldPitchMin_n_v2 = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PS_BldPitchMin) == 0) THEN
            vit_CntrPar_PS_BldPitchMin_n_v2 = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PS_BldPitchMin) <= vit_CntrPar_PS_BldPitchMin_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PS_BldPitchMin_ptr)) THEN
            vit_CntrPar_PS_BldPitchMin_n_v2 = INT(SIZE(vit_local_CntrPar%PS_BldPitchMin), C_INT)
            IF (vit_CntrPar_PS_BldPitchMin_n_v2 > 0) &
                vit_tmp_CntrPar_PS_BldPitchMin(1:vit_CntrPar_PS_BldPitchMin_n_v2) = &
                    vit_local_CntrPar%PS_BldPitchMin(1:vit_CntrPar_PS_BldPitchMin_n_v2)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PS_BldPitchMin came back with ', &
                SIZE(vit_local_CntrPar%PS_BldPitchMin), ' element(s), past the ', &
                vit_CntrPar_PS_BldPitchMin_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_SU_Mode = vit_local_CntrPar%SU_Mode
        vit_CntrPar_SU_StartTime = vit_local_CntrPar%SU_StartTime
        vit_CntrPar_SU_FW_MinDuration = vit_local_CntrPar%SU_FW_MinDuration
        vit_CntrPar_SU_RotorSpeedThresh = vit_local_CntrPar%SU_RotorSpeedThresh
        vit_CntrPar_SU_RotorSpeedCornerFreq = vit_local_CntrPar%SU_RotorSpeedCornerFreq
        vit_CntrPar_SU_LoadStages_N = vit_local_CntrPar%SU_LoadStages_N
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SU_LoadStages)) THEN
            vit_CntrPar_SU_LoadStages_n_v2 = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SU_LoadStages) == 0) THEN
            vit_CntrPar_SU_LoadStages_n_v2 = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SU_LoadStages) <= vit_CntrPar_SU_LoadStages_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SU_LoadStages_ptr)) THEN
            vit_CntrPar_SU_LoadStages_n_v2 = INT(SIZE(vit_local_CntrPar%SU_LoadStages), C_INT)
            IF (vit_CntrPar_SU_LoadStages_n_v2 > 0) &
                vit_tmp_CntrPar_SU_LoadStages(1:vit_CntrPar_SU_LoadStages_n_v2) = &
                    vit_local_CntrPar%SU_LoadStages(1:vit_CntrPar_SU_LoadStages_n_v2)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SU_LoadStages came back with ', &
                SIZE(vit_local_CntrPar%SU_LoadStages), ' element(s), past the ', &
                vit_CntrPar_SU_LoadStages_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SU_LoadRampDuration)) THEN
            vit_CntrPar_SU_LoadRampDuration_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SU_LoadRampDuration) == 0) THEN
            vit_CntrPar_SU_LoadRampDuration_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SU_LoadRampDuration) <= vit_CntrPar_SU_LoadRampDuration_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SU_LoadRampDuration_ptr)) THEN
            vit_CntrPar_SU_LoadRampDuration_n = INT(SIZE(vit_local_CntrPar%SU_LoadRampDuration), C_INT)
            IF (vit_CntrPar_SU_LoadRampDuration_n > 0) &
                vit_tmp_CntrPar_SU_LoadRampDuration(1:vit_CntrPar_SU_LoadRampDuration_n) = &
                    vit_local_CntrPar%SU_LoadRampDuration(1:vit_CntrPar_SU_LoadRampDuration_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SU_LoadRampDuration came back with ', &
                SIZE(vit_local_CntrPar%SU_LoadRampDuration), ' element(s), past the ', &
                vit_CntrPar_SU_LoadRampDuration_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SU_LoadHoldDuration)) THEN
            vit_CntrPar_SU_LoadHoldDuration_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SU_LoadHoldDuration) == 0) THEN
            vit_CntrPar_SU_LoadHoldDuration_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SU_LoadHoldDuration) <= vit_CntrPar_SU_LoadHoldDuration_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SU_LoadHoldDuration_ptr)) THEN
            vit_CntrPar_SU_LoadHoldDuration_n = INT(SIZE(vit_local_CntrPar%SU_LoadHoldDuration), C_INT)
            IF (vit_CntrPar_SU_LoadHoldDuration_n > 0) &
                vit_tmp_CntrPar_SU_LoadHoldDuration(1:vit_CntrPar_SU_LoadHoldDuration_n) = &
                    vit_local_CntrPar%SU_LoadHoldDuration(1:vit_CntrPar_SU_LoadHoldDuration_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SU_LoadHoldDuration came back with ', &
                SIZE(vit_local_CntrPar%SU_LoadHoldDuration), ' element(s), past the ', &
                vit_CntrPar_SU_LoadHoldDuration_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_SD_Mode = vit_local_CntrPar%SD_Mode
        vit_CntrPar_SD_TimeActivate = vit_local_CntrPar%SD_TimeActivate
        vit_CntrPar_SD_EnablePitch = vit_local_CntrPar%SD_EnablePitch
        vit_CntrPar_SD_EnableYawError = vit_local_CntrPar%SD_EnableYawError
        vit_CntrPar_SD_EnableGenSpeed = vit_local_CntrPar%SD_EnableGenSpeed
        vit_CntrPar_SD_EnableTime = vit_local_CntrPar%SD_EnableTime
        vit_CntrPar_SD_MaxPit = vit_local_CntrPar%SD_MaxPit
        vit_CntrPar_SD_PitchCornerFreq = vit_local_CntrPar%SD_PitchCornerFreq
        vit_CntrPar_SD_MaxYawError = vit_local_CntrPar%SD_MaxYawError
        vit_CntrPar_SD_YawErrorCornerFreq = vit_local_CntrPar%SD_YawErrorCornerFreq
        vit_CntrPar_SD_MaxGenSpd = vit_local_CntrPar%SD_MaxGenSpd
        vit_CntrPar_SD_GenSpdCornerFreq = vit_local_CntrPar%SD_GenSpdCornerFreq
        vit_CntrPar_SD_Time = vit_local_CntrPar%SD_Time
        vit_CntrPar_SD_Method = vit_local_CntrPar%SD_Method
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SD_MaxTorqueRate)) THEN
            vit_CntrPar_SD_MaxTorqueRate_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_MaxTorqueRate) == 0) THEN
            vit_CntrPar_SD_MaxTorqueRate_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_MaxTorqueRate) <= vit_CntrPar_SD_MaxTorqueRate_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SD_MaxTorqueRate_ptr)) THEN
            vit_CntrPar_SD_MaxTorqueRate_n = INT(SIZE(vit_local_CntrPar%SD_MaxTorqueRate), C_INT)
            IF (vit_CntrPar_SD_MaxTorqueRate_n > 0) &
                vit_tmp_CntrPar_SD_MaxTorqueRate(1:vit_CntrPar_SD_MaxTorqueRate_n) = &
                    vit_local_CntrPar%SD_MaxTorqueRate(1:vit_CntrPar_SD_MaxTorqueRate_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SD_MaxTorqueRate came back with ', &
                SIZE(vit_local_CntrPar%SD_MaxTorqueRate), ' element(s), past the ', &
                vit_CntrPar_SD_MaxTorqueRate_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SD_MaxPitchRate)) THEN
            vit_CntrPar_SD_MaxPitchRate_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_MaxPitchRate) == 0) THEN
            vit_CntrPar_SD_MaxPitchRate_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_MaxPitchRate) <= vit_CntrPar_SD_MaxPitchRate_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SD_MaxPitchRate_ptr)) THEN
            vit_CntrPar_SD_MaxPitchRate_n = INT(SIZE(vit_local_CntrPar%SD_MaxPitchRate), C_INT)
            IF (vit_CntrPar_SD_MaxPitchRate_n > 0) &
                vit_tmp_CntrPar_SD_MaxPitchRate(1:vit_CntrPar_SD_MaxPitchRate_n) = &
                    vit_local_CntrPar%SD_MaxPitchRate(1:vit_CntrPar_SD_MaxPitchRate_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SD_MaxPitchRate came back with ', &
                SIZE(vit_local_CntrPar%SD_MaxPitchRate), ' element(s), past the ', &
                vit_CntrPar_SD_MaxPitchRate_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SD_StagePitch)) THEN
            vit_CntrPar_SD_StagePitch_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_StagePitch) == 0) THEN
            vit_CntrPar_SD_StagePitch_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_StagePitch) <= vit_CntrPar_SD_StagePitch_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SD_StagePitch_ptr)) THEN
            vit_CntrPar_SD_StagePitch_n = INT(SIZE(vit_local_CntrPar%SD_StagePitch), C_INT)
            IF (vit_CntrPar_SD_StagePitch_n > 0) &
                vit_tmp_CntrPar_SD_StagePitch(1:vit_CntrPar_SD_StagePitch_n) = &
                    vit_local_CntrPar%SD_StagePitch(1:vit_CntrPar_SD_StagePitch_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SD_StagePitch came back with ', &
                SIZE(vit_local_CntrPar%SD_StagePitch), ' element(s), past the ', &
                vit_CntrPar_SD_StagePitch_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%SD_StageTime)) THEN
            vit_CntrPar_SD_StageTime_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_StageTime) == 0) THEN
            vit_CntrPar_SD_StageTime_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%SD_StageTime) <= vit_CntrPar_SD_StageTime_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_SD_StageTime_ptr)) THEN
            vit_CntrPar_SD_StageTime_n = INT(SIZE(vit_local_CntrPar%SD_StageTime), C_INT)
            IF (vit_CntrPar_SD_StageTime_n > 0) &
                vit_tmp_CntrPar_SD_StageTime(1:vit_CntrPar_SD_StageTime_n) = &
                    vit_local_CntrPar%SD_StageTime(1:vit_CntrPar_SD_StageTime_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%SD_StageTime came back with ', &
                SIZE(vit_local_CntrPar%SD_StageTime), ' element(s), past the ', &
                vit_CntrPar_SD_StageTime_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_SD_Stage_N = vit_local_CntrPar%SD_Stage_N
        vit_CntrPar_Fl_Mode = vit_local_CntrPar%Fl_Mode
        vit_CntrPar_Fl_n = vit_local_CntrPar%Fl_n
        IF (.NOT. ALLOCATED(vit_local_CntrPar%Fl_Kp)) THEN
            vit_CntrPar_Fl_Kp_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Fl_Kp) == 0) THEN
            vit_CntrPar_Fl_Kp_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Fl_Kp) <= vit_CntrPar_Fl_Kp_cap .AND. C_ASSOCIATED(vit_CntrPar_Fl_Kp_ptr)) THEN
            vit_CntrPar_Fl_Kp_n = INT(SIZE(vit_local_CntrPar%Fl_Kp), C_INT)
            IF (vit_CntrPar_Fl_Kp_n > 0) &
                vit_tmp_CntrPar_Fl_Kp(1:vit_CntrPar_Fl_Kp_n) = vit_local_CntrPar%Fl_Kp(1:vit_CntrPar_Fl_Kp_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%Fl_Kp came back with ', &
                SIZE(vit_local_CntrPar%Fl_Kp), ' element(s), past the ', &
                vit_CntrPar_Fl_Kp_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%Fl_U)) THEN
            vit_CntrPar_Fl_U_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Fl_U) == 0) THEN
            vit_CntrPar_Fl_U_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Fl_U) <= vit_CntrPar_Fl_U_cap .AND. C_ASSOCIATED(vit_CntrPar_Fl_U_ptr)) THEN
            vit_CntrPar_Fl_U_n = INT(SIZE(vit_local_CntrPar%Fl_U), C_INT)
            IF (vit_CntrPar_Fl_U_n > 0) &
                vit_tmp_CntrPar_Fl_U(1:vit_CntrPar_Fl_U_n) = vit_local_CntrPar%Fl_U(1:vit_CntrPar_Fl_U_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%Fl_U came back with ', &
                SIZE(vit_local_CntrPar%Fl_U), ' element(s), past the ', &
                vit_CntrPar_Fl_U_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_Flp_Mode = vit_local_CntrPar%Flp_Mode
        vit_CntrPar_Flp_Angle = vit_local_CntrPar%Flp_Angle
        vit_CntrPar_Flp_Kp = vit_local_CntrPar%Flp_Kp
        vit_CntrPar_Flp_Ki = vit_local_CntrPar%Flp_Ki
        vit_CntrPar_Flp_MaxPit = vit_local_CntrPar%Flp_MaxPit
        DO vit_ci_CntrPar_OL_Filename = 1, 1024
            vit_CntrPar_OL_Filename(vit_ci_CntrPar_OL_Filename) = &
                vit_local_CntrPar%OL_Filename(vit_ci_CntrPar_OL_Filename:vit_ci_CntrPar_OL_Filename)
        END DO
        vit_CntrPar_OL_Mode = vit_local_CntrPar%OL_Mode
        vit_CntrPar_OL_BP_Mode = vit_local_CntrPar%OL_BP_Mode
        vit_CntrPar_OL_BP_FiltFreq = vit_local_CntrPar%OL_BP_FiltFreq
        vit_CntrPar_Ind_Breakpoint = vit_local_CntrPar%Ind_Breakpoint
        IF (.NOT. ALLOCATED(vit_local_CntrPar%Ind_BldPitch)) THEN
            vit_CntrPar_Ind_BldPitch_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Ind_BldPitch) == 0) THEN
            vit_CntrPar_Ind_BldPitch_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Ind_BldPitch) <= vit_CntrPar_Ind_BldPitch_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_Ind_BldPitch_ptr)) THEN
            vit_CntrPar_Ind_BldPitch_n = INT(SIZE(vit_local_CntrPar%Ind_BldPitch), C_INT)
            IF (vit_CntrPar_Ind_BldPitch_n > 0) &
                vit_tmp_CntrPar_Ind_BldPitch(1:vit_CntrPar_Ind_BldPitch_n) = &
                    vit_local_CntrPar%Ind_BldPitch(1:vit_CntrPar_Ind_BldPitch_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%Ind_BldPitch came back with ', &
                SIZE(vit_local_CntrPar%Ind_BldPitch), ' element(s), past the ', &
                vit_CntrPar_Ind_BldPitch_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_Ind_GenTq = vit_local_CntrPar%Ind_GenTq
        vit_CntrPar_Ind_YawRate = vit_local_CntrPar%Ind_YawRate
        vit_CntrPar_Ind_R_Speed = vit_local_CntrPar%Ind_R_Speed
        vit_CntrPar_Ind_R_Torque = vit_local_CntrPar%Ind_R_Torque
        vit_CntrPar_Ind_R_Pitch = vit_local_CntrPar%Ind_R_Pitch
        vit_CntrPar_Ind_Azimuth = vit_local_CntrPar%Ind_Azimuth
        IF (.NOT. ALLOCATED(vit_local_CntrPar%RP_Gains)) THEN
            vit_CntrPar_RP_Gains_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%RP_Gains) == 0) THEN
            vit_CntrPar_RP_Gains_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%RP_Gains) <= vit_CntrPar_RP_Gains_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_RP_Gains_ptr)) THEN
            vit_CntrPar_RP_Gains_n = INT(SIZE(vit_local_CntrPar%RP_Gains), C_INT)
            IF (vit_CntrPar_RP_Gains_n > 0) &
                vit_tmp_CntrPar_RP_Gains(1:vit_CntrPar_RP_Gains_n) = &
                    vit_local_CntrPar%RP_Gains(1:vit_CntrPar_RP_Gains_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%RP_Gains came back with ', &
                SIZE(vit_local_CntrPar%RP_Gains), ' element(s), past the ', &
                vit_CntrPar_RP_Gains_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%Ind_CableControl)) THEN
            vit_CntrPar_Ind_CableControl_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Ind_CableControl) == 0) THEN
            vit_CntrPar_Ind_CableControl_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Ind_CableControl) <= vit_CntrPar_Ind_CableControl_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_Ind_CableControl_ptr)) THEN
            vit_CntrPar_Ind_CableControl_n = INT(SIZE(vit_local_CntrPar%Ind_CableControl), C_INT)
            IF (vit_CntrPar_Ind_CableControl_n > 0) &
                vit_tmp_CntrPar_Ind_CableControl(1:vit_CntrPar_Ind_CableControl_n) = &
                    vit_local_CntrPar%Ind_CableControl(1:vit_CntrPar_Ind_CableControl_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%Ind_CableControl came back with ', &
                SIZE(vit_local_CntrPar%Ind_CableControl), ' element(s), past the ', &
                vit_CntrPar_Ind_CableControl_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%Ind_StructControl)) THEN
            vit_CntrPar_Ind_StructControl_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Ind_StructControl) == 0) THEN
            vit_CntrPar_Ind_StructControl_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%Ind_StructControl) <= vit_CntrPar_Ind_StructControl_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_Ind_StructControl_ptr)) THEN
            vit_CntrPar_Ind_StructControl_n = INT(SIZE(vit_local_CntrPar%Ind_StructControl), C_INT)
            IF (vit_CntrPar_Ind_StructControl_n > 0) &
                vit_tmp_CntrPar_Ind_StructControl(1:vit_CntrPar_Ind_StructControl_n) = &
                    vit_local_CntrPar%Ind_StructControl(1:vit_CntrPar_Ind_StructControl_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%Ind_StructControl came back with ', &
                SIZE(vit_local_CntrPar%Ind_StructControl), ' element(s), past the ', &
                vit_CntrPar_Ind_StructControl_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_Breakpoints)) THEN
            vit_CntrPar_OL_Breakpoints_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_Breakpoints) == 0) THEN
            vit_CntrPar_OL_Breakpoints_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_Breakpoints) <= vit_CntrPar_OL_Breakpoints_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_Breakpoints_ptr)) THEN
            vit_CntrPar_OL_Breakpoints_n = INT(SIZE(vit_local_CntrPar%OL_Breakpoints), C_INT)
            IF (vit_CntrPar_OL_Breakpoints_n > 0) &
                vit_tmp_CntrPar_OL_Breakpoints(1:vit_CntrPar_OL_Breakpoints_n) = &
                    vit_local_CntrPar%OL_Breakpoints(1:vit_CntrPar_OL_Breakpoints_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_Breakpoints came back with ', &
                SIZE(vit_local_CntrPar%OL_Breakpoints), ' element(s), past the ', &
                vit_CntrPar_OL_Breakpoints_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_BldPitch1)) THEN
            vit_CntrPar_OL_BldPitch1_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_BldPitch1) == 0) THEN
            vit_CntrPar_OL_BldPitch1_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_BldPitch1) <= vit_CntrPar_OL_BldPitch1_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_BldPitch1_ptr)) THEN
            vit_CntrPar_OL_BldPitch1_n = INT(SIZE(vit_local_CntrPar%OL_BldPitch1), C_INT)
            IF (vit_CntrPar_OL_BldPitch1_n > 0) &
                vit_tmp_CntrPar_OL_BldPitch1(1:vit_CntrPar_OL_BldPitch1_n) = &
                    vit_local_CntrPar%OL_BldPitch1(1:vit_CntrPar_OL_BldPitch1_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_BldPitch1 came back with ', &
                SIZE(vit_local_CntrPar%OL_BldPitch1), ' element(s), past the ', &
                vit_CntrPar_OL_BldPitch1_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_BldPitch2)) THEN
            vit_CntrPar_OL_BldPitch2_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_BldPitch2) == 0) THEN
            vit_CntrPar_OL_BldPitch2_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_BldPitch2) <= vit_CntrPar_OL_BldPitch2_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_BldPitch2_ptr)) THEN
            vit_CntrPar_OL_BldPitch2_n = INT(SIZE(vit_local_CntrPar%OL_BldPitch2), C_INT)
            IF (vit_CntrPar_OL_BldPitch2_n > 0) &
                vit_tmp_CntrPar_OL_BldPitch2(1:vit_CntrPar_OL_BldPitch2_n) = &
                    vit_local_CntrPar%OL_BldPitch2(1:vit_CntrPar_OL_BldPitch2_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_BldPitch2 came back with ', &
                SIZE(vit_local_CntrPar%OL_BldPitch2), ' element(s), past the ', &
                vit_CntrPar_OL_BldPitch2_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_BldPitch3)) THEN
            vit_CntrPar_OL_BldPitch3_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_BldPitch3) == 0) THEN
            vit_CntrPar_OL_BldPitch3_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_BldPitch3) <= vit_CntrPar_OL_BldPitch3_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_BldPitch3_ptr)) THEN
            vit_CntrPar_OL_BldPitch3_n = INT(SIZE(vit_local_CntrPar%OL_BldPitch3), C_INT)
            IF (vit_CntrPar_OL_BldPitch3_n > 0) &
                vit_tmp_CntrPar_OL_BldPitch3(1:vit_CntrPar_OL_BldPitch3_n) = &
                    vit_local_CntrPar%OL_BldPitch3(1:vit_CntrPar_OL_BldPitch3_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_BldPitch3 came back with ', &
                SIZE(vit_local_CntrPar%OL_BldPitch3), ' element(s), past the ', &
                vit_CntrPar_OL_BldPitch3_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (ALLOCATED(vit_local_CntrPar%OL_CableControl) .AND. C_ASSOCIATED(vit_CntrPar_OL_CableControl_ptr)) THEN
            IF (SIZE(vit_local_CntrPar%OL_CableControl, &
                1) == vit_CntrPar_OL_CableControl_rows .AND. SIZE(vit_local_CntrPar%OL_CableControl, &
                2) == vit_CntrPar_OL_CableControl_cols) THEN
                DO vit_j_OL_CableControl = 1, vit_CntrPar_OL_CableControl_cols
                    DO vit_i_OL_CableControl = 1, vit_CntrPar_OL_CableControl_rows
                        vit_tmp_CntrPar_OL_CableControl((vit_j_OL_CableControl-1)*vit_CntrPar_OL_CableControl_rows + &
                            vit_i_OL_CableControl) = &
                            vit_local_CntrPar%OL_CableControl(vit_i_OL_CableControl, vit_j_OL_CableControl)
                    END DO
                END DO
            ELSE
                WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                    'VIT bridge: CntrPar%OL_CableControl came back ', &
                    SIZE(vit_local_CntrPar%OL_CableControl, 1), ' x ', &
                    SIZE(vit_local_CntrPar%OL_CableControl, 2), &
                    ', not the shape the caller stated; not copied back'
            END IF
        END IF
        IF (ALLOCATED(vit_local_CntrPar%OL_StructControl) .AND. C_ASSOCIATED(vit_CntrPar_OL_StructControl_ptr)) THEN
            IF (SIZE(vit_local_CntrPar%OL_StructControl, &
                1) == vit_CntrPar_OL_StructControl_rows .AND. SIZE(vit_local_CntrPar%OL_StructControl, &
                2) == vit_CntrPar_OL_StructControl_cols) THEN
                DO vit_j_OL_StructControl = 1, vit_CntrPar_OL_StructControl_cols
                    DO vit_i_OL_StructControl = 1, vit_CntrPar_OL_StructControl_rows
                        vit_tmp_CntrPar_OL_StructControl((vit_j_OL_StructControl-1)*vit_CntrPar_OL_StructControl_rows &
                            + vit_i_OL_StructControl) = &
                            vit_local_CntrPar%OL_StructControl(vit_i_OL_StructControl, vit_j_OL_StructControl)
                    END DO
                END DO
            ELSE
                WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                    'VIT bridge: CntrPar%OL_StructControl came back ', &
                    SIZE(vit_local_CntrPar%OL_StructControl, 1), ' x ', &
                    SIZE(vit_local_CntrPar%OL_StructControl, 2), &
                    ', not the shape the caller stated; not copied back'
            END IF
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_GenTq)) THEN
            vit_CntrPar_OL_GenTq_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_GenTq) == 0) THEN
            vit_CntrPar_OL_GenTq_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_GenTq) <= vit_CntrPar_OL_GenTq_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_GenTq_ptr)) THEN
            vit_CntrPar_OL_GenTq_n = INT(SIZE(vit_local_CntrPar%OL_GenTq), C_INT)
            IF (vit_CntrPar_OL_GenTq_n > 0) &
                vit_tmp_CntrPar_OL_GenTq(1:vit_CntrPar_OL_GenTq_n) = &
                    vit_local_CntrPar%OL_GenTq(1:vit_CntrPar_OL_GenTq_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_GenTq came back with ', &
                SIZE(vit_local_CntrPar%OL_GenTq), ' element(s), past the ', &
                vit_CntrPar_OL_GenTq_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_YawRate)) THEN
            vit_CntrPar_OL_YawRate_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_YawRate) == 0) THEN
            vit_CntrPar_OL_YawRate_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_YawRate) <= vit_CntrPar_OL_YawRate_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_YawRate_ptr)) THEN
            vit_CntrPar_OL_YawRate_n = INT(SIZE(vit_local_CntrPar%OL_YawRate), C_INT)
            IF (vit_CntrPar_OL_YawRate_n > 0) &
                vit_tmp_CntrPar_OL_YawRate(1:vit_CntrPar_OL_YawRate_n) = &
                    vit_local_CntrPar%OL_YawRate(1:vit_CntrPar_OL_YawRate_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_YawRate came back with ', &
                SIZE(vit_local_CntrPar%OL_YawRate), ' element(s), past the ', &
                vit_CntrPar_OL_YawRate_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_Azimuth)) THEN
            vit_CntrPar_OL_Azimuth_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_Azimuth) == 0) THEN
            vit_CntrPar_OL_Azimuth_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_Azimuth) <= vit_CntrPar_OL_Azimuth_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_Azimuth_ptr)) THEN
            vit_CntrPar_OL_Azimuth_n = INT(SIZE(vit_local_CntrPar%OL_Azimuth), C_INT)
            IF (vit_CntrPar_OL_Azimuth_n > 0) &
                vit_tmp_CntrPar_OL_Azimuth(1:vit_CntrPar_OL_Azimuth_n) = &
                    vit_local_CntrPar%OL_Azimuth(1:vit_CntrPar_OL_Azimuth_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_Azimuth came back with ', &
                SIZE(vit_local_CntrPar%OL_Azimuth), ' element(s), past the ', &
                vit_CntrPar_OL_Azimuth_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_R_Speed)) THEN
            vit_CntrPar_OL_R_Speed_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_R_Speed) == 0) THEN
            vit_CntrPar_OL_R_Speed_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_R_Speed) <= vit_CntrPar_OL_R_Speed_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_R_Speed_ptr)) THEN
            vit_CntrPar_OL_R_Speed_n = INT(SIZE(vit_local_CntrPar%OL_R_Speed), C_INT)
            IF (vit_CntrPar_OL_R_Speed_n > 0) &
                vit_tmp_CntrPar_OL_R_Speed(1:vit_CntrPar_OL_R_Speed_n) = &
                    vit_local_CntrPar%OL_R_Speed(1:vit_CntrPar_OL_R_Speed_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_R_Speed came back with ', &
                SIZE(vit_local_CntrPar%OL_R_Speed), ' element(s), past the ', &
                vit_CntrPar_OL_R_Speed_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_R_Torque)) THEN
            vit_CntrPar_OL_R_Torque_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_R_Torque) == 0) THEN
            vit_CntrPar_OL_R_Torque_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_R_Torque) <= vit_CntrPar_OL_R_Torque_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_R_Torque_ptr)) THEN
            vit_CntrPar_OL_R_Torque_n = INT(SIZE(vit_local_CntrPar%OL_R_Torque), C_INT)
            IF (vit_CntrPar_OL_R_Torque_n > 0) &
                vit_tmp_CntrPar_OL_R_Torque(1:vit_CntrPar_OL_R_Torque_n) = &
                    vit_local_CntrPar%OL_R_Torque(1:vit_CntrPar_OL_R_Torque_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_R_Torque came back with ', &
                SIZE(vit_local_CntrPar%OL_R_Torque), ' element(s), past the ', &
                vit_CntrPar_OL_R_Torque_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%OL_R_Pitch)) THEN
            vit_CntrPar_OL_R_Pitch_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_R_Pitch) == 0) THEN
            vit_CntrPar_OL_R_Pitch_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%OL_R_Pitch) <= vit_CntrPar_OL_R_Pitch_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_OL_R_Pitch_ptr)) THEN
            vit_CntrPar_OL_R_Pitch_n = INT(SIZE(vit_local_CntrPar%OL_R_Pitch), C_INT)
            IF (vit_CntrPar_OL_R_Pitch_n > 0) &
                vit_tmp_CntrPar_OL_R_Pitch(1:vit_CntrPar_OL_R_Pitch_n) = &
                    vit_local_CntrPar%OL_R_Pitch(1:vit_CntrPar_OL_R_Pitch_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%OL_R_Pitch came back with ', &
                SIZE(vit_local_CntrPar%OL_R_Pitch), ' element(s), past the ', &
                vit_CntrPar_OL_R_Pitch_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (ALLOCATED(vit_local_CntrPar%OL_Channels) .AND. C_ASSOCIATED(vit_CntrPar_OL_Channels_ptr)) THEN
            IF (SIZE(vit_local_CntrPar%OL_Channels, &
                1) == vit_CntrPar_OL_Channels_rows .AND. SIZE(vit_local_CntrPar%OL_Channels, &
                2) == vit_CntrPar_OL_Channels_cols) THEN
                DO vit_j_OL_Channels = 1, vit_CntrPar_OL_Channels_cols
                    DO vit_i_OL_Channels = 1, vit_CntrPar_OL_Channels_rows
                        vit_tmp_CntrPar_OL_Channels((vit_j_OL_Channels-1)*vit_CntrPar_OL_Channels_rows + &
                            vit_i_OL_Channels) = &
                            vit_local_CntrPar%OL_Channels(vit_i_OL_Channels, vit_j_OL_Channels)
                    END DO
                END DO
            ELSE
                WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                    'VIT bridge: CntrPar%OL_Channels came back ', &
                    SIZE(vit_local_CntrPar%OL_Channels, 1), ' x ', &
                    SIZE(vit_local_CntrPar%OL_Channels, 2), &
                    ', not the shape the caller stated; not copied back'
            END IF
        END IF
        vit_CntrPar_PA_Mode = vit_local_CntrPar%PA_Mode
        vit_CntrPar_PA_CornerFreq = vit_local_CntrPar%PA_CornerFreq
        vit_CntrPar_PA_Damping = vit_local_CntrPar%PA_Damping
        vit_CntrPar_AWC_Mode = vit_local_CntrPar%AWC_Mode
        vit_CntrPar_AWC_NumModes = vit_local_CntrPar%AWC_NumModes
        IF (.NOT. ALLOCATED(vit_local_CntrPar%AWC_n)) THEN
            vit_CntrPar_AWC_n_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_n) == 0) THEN
            vit_CntrPar_AWC_n_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_n) <= vit_CntrPar_AWC_n_cap .AND. C_ASSOCIATED(vit_CntrPar_AWC_n_ptr)) THEN
            vit_CntrPar_AWC_n_n = INT(SIZE(vit_local_CntrPar%AWC_n), C_INT)
            IF (vit_CntrPar_AWC_n_n > 0) &
                vit_tmp_CntrPar_AWC_n(1:vit_CntrPar_AWC_n_n) = vit_local_CntrPar%AWC_n(1:vit_CntrPar_AWC_n_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%AWC_n came back with ', &
                SIZE(vit_local_CntrPar%AWC_n), ' element(s), past the ', &
                vit_CntrPar_AWC_n_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%AWC_harmonic)) THEN
            vit_CntrPar_AWC_harmonic_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_harmonic) == 0) THEN
            vit_CntrPar_AWC_harmonic_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_harmonic) <= vit_CntrPar_AWC_harmonic_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_AWC_harmonic_ptr)) THEN
            vit_CntrPar_AWC_harmonic_n = INT(SIZE(vit_local_CntrPar%AWC_harmonic), C_INT)
            IF (vit_CntrPar_AWC_harmonic_n > 0) &
                vit_tmp_CntrPar_AWC_harmonic(1:vit_CntrPar_AWC_harmonic_n) = &
                    vit_local_CntrPar%AWC_harmonic(1:vit_CntrPar_AWC_harmonic_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%AWC_harmonic came back with ', &
                SIZE(vit_local_CntrPar%AWC_harmonic), ' element(s), past the ', &
                vit_CntrPar_AWC_harmonic_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%AWC_freq)) THEN
            vit_CntrPar_AWC_freq_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_freq) == 0) THEN
            vit_CntrPar_AWC_freq_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_freq) <= vit_CntrPar_AWC_freq_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_AWC_freq_ptr)) THEN
            vit_CntrPar_AWC_freq_n = INT(SIZE(vit_local_CntrPar%AWC_freq), C_INT)
            IF (vit_CntrPar_AWC_freq_n > 0) &
                vit_tmp_CntrPar_AWC_freq(1:vit_CntrPar_AWC_freq_n) = &
                    vit_local_CntrPar%AWC_freq(1:vit_CntrPar_AWC_freq_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%AWC_freq came back with ', &
                SIZE(vit_local_CntrPar%AWC_freq), ' element(s), past the ', &
                vit_CntrPar_AWC_freq_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%AWC_amp)) THEN
            vit_CntrPar_AWC_amp_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_amp) == 0) THEN
            vit_CntrPar_AWC_amp_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_amp) <= vit_CntrPar_AWC_amp_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_AWC_amp_ptr)) THEN
            vit_CntrPar_AWC_amp_n = INT(SIZE(vit_local_CntrPar%AWC_amp), C_INT)
            IF (vit_CntrPar_AWC_amp_n > 0) &
                vit_tmp_CntrPar_AWC_amp(1:vit_CntrPar_AWC_amp_n) = vit_local_CntrPar%AWC_amp(1:vit_CntrPar_AWC_amp_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%AWC_amp came back with ', &
                SIZE(vit_local_CntrPar%AWC_amp), ' element(s), past the ', &
                vit_CntrPar_AWC_amp_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%AWC_clockangle)) THEN
            vit_CntrPar_AWC_clockangle_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_clockangle) == 0) THEN
            vit_CntrPar_AWC_clockangle_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_clockangle) <= vit_CntrPar_AWC_clockangle_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_AWC_clockangle_ptr)) THEN
            vit_CntrPar_AWC_clockangle_n = INT(SIZE(vit_local_CntrPar%AWC_clockangle), C_INT)
            IF (vit_CntrPar_AWC_clockangle_n > 0) &
                vit_tmp_CntrPar_AWC_clockangle(1:vit_CntrPar_AWC_clockangle_n) = &
                    vit_local_CntrPar%AWC_clockangle(1:vit_CntrPar_AWC_clockangle_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%AWC_clockangle came back with ', &
                SIZE(vit_local_CntrPar%AWC_clockangle), ' element(s), past the ', &
                vit_CntrPar_AWC_clockangle_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_AWC_phaseoffset = vit_local_CntrPar%AWC_phaseoffset
        IF (.NOT. ALLOCATED(vit_local_CntrPar%AWC_CntrGains)) THEN
            vit_CntrPar_AWC_CntrGains_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_CntrGains) == 0) THEN
            vit_CntrPar_AWC_CntrGains_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%AWC_CntrGains) <= vit_CntrPar_AWC_CntrGains_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_AWC_CntrGains_ptr)) THEN
            vit_CntrPar_AWC_CntrGains_n = INT(SIZE(vit_local_CntrPar%AWC_CntrGains), C_INT)
            IF (vit_CntrPar_AWC_CntrGains_n > 0) &
                vit_tmp_CntrPar_AWC_CntrGains(1:vit_CntrPar_AWC_CntrGains_n) = &
                    vit_local_CntrPar%AWC_CntrGains(1:vit_CntrPar_AWC_CntrGains_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%AWC_CntrGains came back with ', &
                SIZE(vit_local_CntrPar%AWC_CntrGains), ' element(s), past the ', &
                vit_CntrPar_AWC_CntrGains_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_PF_Mode = vit_local_CntrPar%PF_Mode
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PF_Offsets)) THEN
            vit_CntrPar_PF_Offsets_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PF_Offsets) == 0) THEN
            vit_CntrPar_PF_Offsets_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PF_Offsets) <= vit_CntrPar_PF_Offsets_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PF_Offsets_ptr)) THEN
            vit_CntrPar_PF_Offsets_n = INT(SIZE(vit_local_CntrPar%PF_Offsets), C_INT)
            IF (vit_CntrPar_PF_Offsets_n > 0) &
                vit_tmp_CntrPar_PF_Offsets(1:vit_CntrPar_PF_Offsets_n) = &
                    vit_local_CntrPar%PF_Offsets(1:vit_CntrPar_PF_Offsets_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PF_Offsets came back with ', &
                SIZE(vit_local_CntrPar%PF_Offsets), ' element(s), past the ', &
                vit_CntrPar_PF_Offsets_cap, '-element buffer the caller supplied; not copied back'
        END IF
        IF (.NOT. ALLOCATED(vit_local_CntrPar%PF_TimeStuck)) THEN
            vit_CntrPar_PF_TimeStuck_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PF_TimeStuck) == 0) THEN
            vit_CntrPar_PF_TimeStuck_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%PF_TimeStuck) <= vit_CntrPar_PF_TimeStuck_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_PF_TimeStuck_ptr)) THEN
            vit_CntrPar_PF_TimeStuck_n = INT(SIZE(vit_local_CntrPar%PF_TimeStuck), C_INT)
            IF (vit_CntrPar_PF_TimeStuck_n > 0) &
                vit_tmp_CntrPar_PF_TimeStuck(1:vit_CntrPar_PF_TimeStuck_n) = &
                    vit_local_CntrPar%PF_TimeStuck(1:vit_CntrPar_PF_TimeStuck_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%PF_TimeStuck came back with ', &
                SIZE(vit_local_CntrPar%PF_TimeStuck), ' element(s), past the ', &
                vit_CntrPar_PF_TimeStuck_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_Ext_Mode = vit_local_CntrPar%Ext_Mode
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
        vit_CntrPar_ZMQ_Mode = vit_local_CntrPar%ZMQ_Mode
        DO vit_ci_CntrPar_ZMQ_CommAddress = 1, 256
            vit_CntrPar_ZMQ_CommAddress(vit_ci_CntrPar_ZMQ_CommAddress) = &
                vit_local_CntrPar%ZMQ_CommAddress(vit_ci_CntrPar_ZMQ_CommAddress:vit_ci_CntrPar_ZMQ_CommAddress)
        END DO
        vit_CntrPar_ZMQ_UpdatePeriod = vit_local_CntrPar%ZMQ_UpdatePeriod
        vit_CntrPar_CC_Mode = vit_local_CntrPar%CC_Mode
        vit_CntrPar_CC_Group_N = vit_local_CntrPar%CC_Group_N
        vit_CntrPar_CC_ActTau = vit_local_CntrPar%CC_ActTau
        IF (.NOT. ALLOCATED(vit_local_CntrPar%CC_GroupIndex)) THEN
            vit_CntrPar_CC_GroupIndex_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%CC_GroupIndex) == 0) THEN
            vit_CntrPar_CC_GroupIndex_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%CC_GroupIndex) <= vit_CntrPar_CC_GroupIndex_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_CC_GroupIndex_ptr)) THEN
            vit_CntrPar_CC_GroupIndex_n = INT(SIZE(vit_local_CntrPar%CC_GroupIndex), C_INT)
            IF (vit_CntrPar_CC_GroupIndex_n > 0) &
                vit_tmp_CntrPar_CC_GroupIndex(1:vit_CntrPar_CC_GroupIndex_n) = &
                    vit_local_CntrPar%CC_GroupIndex(1:vit_CntrPar_CC_GroupIndex_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%CC_GroupIndex came back with ', &
                SIZE(vit_local_CntrPar%CC_GroupIndex), ' element(s), past the ', &
                vit_CntrPar_CC_GroupIndex_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_StC_Mode = vit_local_CntrPar%StC_Mode
        vit_CntrPar_StC_Group_N = vit_local_CntrPar%StC_Group_N
        IF (.NOT. ALLOCATED(vit_local_CntrPar%StC_GroupIndex)) THEN
            vit_CntrPar_StC_GroupIndex_n = -1_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%StC_GroupIndex) == 0) THEN
            vit_CntrPar_StC_GroupIndex_n = 0_C_INT
        ELSE IF (SIZE(vit_local_CntrPar%StC_GroupIndex) <= vit_CntrPar_StC_GroupIndex_cap .AND. &
            C_ASSOCIATED(vit_CntrPar_StC_GroupIndex_ptr)) THEN
            vit_CntrPar_StC_GroupIndex_n = INT(SIZE(vit_local_CntrPar%StC_GroupIndex), C_INT)
            IF (vit_CntrPar_StC_GroupIndex_n > 0) &
                vit_tmp_CntrPar_StC_GroupIndex(1:vit_CntrPar_StC_GroupIndex_n) = &
                    vit_local_CntrPar%StC_GroupIndex(1:vit_CntrPar_StC_GroupIndex_n)
        ELSE
            WRITE(ERROR_UNIT,'(A,I0,A,I0,A)') &
                'VIT bridge: CntrPar%StC_GroupIndex came back with ', &
                SIZE(vit_local_CntrPar%StC_GroupIndex), ' element(s), past the ', &
                vit_CntrPar_StC_GroupIndex_cap, '-element buffer the caller supplied; not copied back'
        END IF
        vit_CntrPar_PC_RtTq99 = vit_local_CntrPar%PC_RtTq99
        vit_CntrPar_VS_MaxOMTq = vit_local_CntrPar%VS_MaxOMTq
        vit_CntrPar_VS_MinOMTq = vit_local_CntrPar%VS_MinOMTq
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
        vit_ErrVar_size_avcMSG = vit_local_ErrVar%size_avcMSG
        vit_ErrVar_aviFAIL = vit_local_ErrVar%aviFAIL
        vit_ErrVar_ErrStat = vit_local_ErrVar%ErrStat
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
    END SUBROUTINE updatezeromq_f90