// ROSCO Constants — C++ equivalent of Constants.f90
// Source of truth for all mathematical and control mode constants
// used by C++ translations. Values match Constants.f90 exactly.

#ifndef ROSCO_CONSTANTS_H
#define ROSCO_CONSTANTS_H

// Mathematical constants (DbKi = double)
static constexpr double PI      = 3.14159265359;
static constexpr double D2R     = 0.01745329251;     // degrees to radians
static constexpr double R2D     = 57.2957795130;      // radians to degrees
static constexpr double RPS2RPM = 9.5492966;           // rad/s to RPM

// Variable Speed Control Modes (VS_ControlMode)
static constexpr int VS_Mode_Disabled   = 0;
static constexpr int VS_Mode_KOmega     = 1;
static constexpr int VS_Mode_WSE_TSR    = 2;
static constexpr int VS_Mode_Power_TSR  = 3;
static constexpr int VS_Mode_Torque_TSR = 4;

// Variable Speed Constant Power Modes (VS_ConstPower)
static constexpr int VS_Mode_ConstTrq = 0;
static constexpr int VS_Mode_ConstPwr = 1;

// Variable Speed Feedback Path Modes (VS_FBP)
static constexpr int VS_FBP_Variable_Pitch  = 0;
static constexpr int VS_FBP_Power_Overspeed = 1;
static constexpr int VS_FBP_WSE_Ref         = 2;
static constexpr int VS_FBP_Torque_Ref      = 3;

// Variable Speed States
static constexpr int VS_State_Error             = 0;
static constexpr int VS_State_Region_1_5        = 1;
static constexpr int VS_State_Region_2          = 2;
static constexpr int VS_State_Region_2_5        = 3;
static constexpr int VS_State_Region_3_ConstTrq = 4;
static constexpr int VS_State_Region_3_ConstPwr = 5;
static constexpr int VS_State_Region_3_FBP      = 6;
static constexpr int VS_State_PI                = 7;

// Pitch Control States
static constexpr int PC_State_Disabled = 0;
static constexpr int PC_State_Enabled  = 1;

// Power Reference Communication Modes
static constexpr int PRC_Comm_Constant = 0;
static constexpr int PRC_Comm_OpenLoop = 1;
static constexpr int PRC_Comm_ZMQ      = 2;

// Rotational Harmonics
static constexpr int NP_1 = 1;
static constexpr int NP_2 = 2;

#endif // ROSCO_CONSTANTS_H
