// VIT Translation
// Function: ReadControlParameterFileSub
// Source: ReadSetParameters.f90
// Module: ReadSetParameters
// Fortran: SUBROUTINE ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, accINFILE_size, RootName, ErrVar)
// Status: unverified
//
// Two-pass DISCON.IN config file parser.
// Pass 1: reads all scalars, counts OL file rows
// Pass 2: fills ALLOCATABLE arrays, computes derived constants, loads OL CSV

#include "vit_types.h"
#include "rosco_constants.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cctype>
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void setError(errorvariables_t* ErrVar, const char* msg) {
    ErrVar->aviFAIL = -1;
    std::memset(ErrVar->ErrMsg, ' ', 1024);
    size_t len = std::strlen(msg);
    if (len > 1024) len = 1024;
    std::memcpy(ErrVar->ErrMsg, msg, len);
}

static std::string trimFortranString(const char* s, int maxLen) {
    int len = maxLen;
    while (len > 0 && s[len - 1] == ' ') len--;
    return std::string(s, len);
}

static void setFortranString(char* dest, int maxLen, const std::string& src) {
    std::memset(dest, ' ', maxLen);
    size_t len = src.size();
    if (len > (size_t)maxLen) len = maxLen;
    std::memcpy(dest, src.c_str(), len);
}

static std::string toUpper(const std::string& s) {
    std::string r = s;
    for (auto& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

// Strip surrounding double quotes from a string value
static std::string stripQuotes(const std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

// Unwrap an angle array in-place (add/subtract 2*PI to keep consecutive
// differences in [-PI, PI]).  Matches Fortran unwrap() semantics.
static void unwrap(double* arr, int n) {
    for (int i = 1; i < n; i++) {
        while (arr[i] - arr[i - 1] <= -PI) {
            for (int j = i; j < n; j++) arr[j] += 2.0 * PI;
        }
        while (arr[i] - arr[i - 1] >= PI) {
            for (int j = i; j < n; j++) arr[j] -= 2.0 * PI;
        }
    }
}

// ---------------------------------------------------------------------------
// DisconParser — reads a DISCON.IN parameter file
// ---------------------------------------------------------------------------

class DisconParser {
    std::vector<std::string> lines_;
    std::string filename_;
    errorvariables_t* errVar_;

    // Find line index where param name matches (case-insensitive).
    // Returns -1 if not found.
    int findParam(const std::string& name) {
        std::string nameUp = toUpper(name);
        for (int i = 0; i < (int)lines_.size(); i++) {
            const std::string& line = lines_[i];
            // Skip pure comment lines (first non-space char is '!')
            size_t firstNonSpace = line.find_first_not_of(" \t");
            if (firstNonSpace == std::string::npos) continue;
            if (line[firstNonSpace] == '!') continue;

            // Find the '!' separator
            size_t bangPos = line.find('!');
            if (bangPos == std::string::npos) continue;

            // Extract the first word after '!'
            size_t wordStart = bangPos + 1;
            while (wordStart < line.size() && (line[wordStart] == ' ' || line[wordStart] == '\t'))
                wordStart++;
            size_t wordEnd = wordStart;
            while (wordEnd < line.size() && line[wordEnd] != ' ' && line[wordEnd] != '\t' &&
                   line[wordEnd] != '-' && line[wordEnd] != '!')
                wordEnd++;
            std::string paramName = line.substr(wordStart, wordEnd - wordStart);
            if (toUpper(paramName) == nameUp) {
                return i;
            }
        }
        return -1;
    }

    // Extract the value part of a line (everything before first '!')
    std::string getValuePart(int idx) {
        const std::string& line = lines_[idx];
        size_t bangPos = line.find('!');
        if (bangPos == std::string::npos) return line;
        return line.substr(0, bangPos);
    }

public:
    bool load(const std::string& filename, errorvariables_t* err) {
        filename_ = filename;
        errVar_ = err;
        lines_.clear();
        std::ifstream f(filename);
        if (!f.is_open()) {
            std::string msg = "Cannot open file: " + filename;
            setError(err, msg.c_str());
            return false;
        }
        std::string line;
        while (std::getline(f, line)) {
            lines_.push_back(line);
        }
        return true;
    }

    // Parse a double scalar
    bool parseDbl(const char* name, double& val, bool allowDefault = true) {
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) {
                val = 0.0;
                return true;
            }
            std::string msg = std::string("Could not find parameter ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::string vp = getValuePart(idx);
        std::istringstream iss(vp);
        if (!(iss >> val)) {
            if (allowDefault) {
                val = 0.0;
                return true;
            }
            std::string msg = std::string("Error reading value for ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        return true;
    }

    // Parse an integer scalar
    bool parseInt(const char* name, int& val, bool allowDefault = true) {
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) {
                val = 0;
                return true;
            }
            std::string msg = std::string("Could not find parameter ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::string vp = getValuePart(idx);
        std::istringstream iss(vp);
        if (!(iss >> val)) {
            if (allowDefault) {
                val = 0;
                return true;
            }
            std::string msg = std::string("Error reading value for ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        return true;
    }

    // Parse a string scalar
    bool parseStr(const char* name, char* val, int maxLen, bool allowDefault = true) {
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) {
                setFortranString(val, maxLen, "unused");
                return true;
            }
            std::string msg = std::string("Could not find parameter ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::string vp = getValuePart(idx);
        std::istringstream iss(vp);
        std::string sv;
        if (!(iss >> sv)) {
            if (allowDefault) {
                setFortranString(val, maxLen, "unused");
                return true;
            }
            std::string msg = std::string("Error reading value for ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        sv = stripQuotes(sv);
        setFortranString(val, maxLen, sv);
        return true;
    }

    // Parse a double array — arr must be pre-allocated with n elements
    bool parseDblAry(const char* name, double* arr, int n, bool allowDefault = true) {
        if (n <= 0) return true;
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) {
                for (int i = 0; i < n; i++) arr[i] = 0.0;
                return true;
            }
            std::string msg = std::string("Could not find parameter ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::string vp = getValuePart(idx);
        std::istringstream iss(vp);
        for (int i = 0; i < n; i++) {
            if (!(iss >> arr[i])) {
                if (allowDefault) {
                    for (int j = i; j < n; j++) arr[j] = 0.0;
                    return true;
                }
                std::string msg = std::string("Error reading array element ") +
                                  std::to_string(i + 1) + " for " + name +
                                  " in " + filename_;
                setError(errVar_, msg.c_str());
                return false;
            }
        }
        return true;
    }

    // Parse an integer array — arr must be pre-allocated with n elements
    bool parseIntAry(const char* name, int* arr, int n, bool allowDefault = true) {
        if (n <= 0) return true;
        int idx = findParam(name);
        if (idx < 0) {
            if (allowDefault) {
                for (int i = 0; i < n; i++) arr[i] = 0;
                return true;
            }
            std::string msg = std::string("Could not find parameter ") + name +
                              " in " + filename_;
            setError(errVar_, msg.c_str());
            return false;
        }
        std::string vp = getValuePart(idx);
        std::istringstream iss(vp);
        for (int i = 0; i < n; i++) {
            if (!(iss >> arr[i])) {
                if (allowDefault) {
                    for (int j = i; j < n; j++) arr[j] = 0;
                    return true;
                }
                std::string msg = std::string("Error reading array element ") +
                                  std::to_string(i + 1) + " for " + name +
                                  " in " + filename_;
                setError(errVar_, msg.c_str());
                return false;
            }
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// pathIsRelative — matches Fortran PathIsRelative
// ---------------------------------------------------------------------------
static bool pathIsRelative(const std::string& path) {
    if (path.empty()) return true;
    // Check for drive letter ":/" or ":\"
    if (path.size() >= 2) {
        if (path[1] == ':' && (path[2] == '/' || path[2] == '\\')) return false;
    }
    // Check for leading "/" or "\"
    if (path[0] == '/' || path[0] == '\\') return false;
    return true;
}

// ---------------------------------------------------------------------------
// Pass 1 — read all scalar fields and count OL rows
// ---------------------------------------------------------------------------

extern "C" {

void readcontrolparameterfilesub_pass1_c(
    controlparameters_view_t* CntrPar,
    localvariables_t* LocalVar,
    const char* filename,
    const char* priPath,
    errorvariables_t* ErrVar,
    int32_t* n_OL_rows,
    int32_t* OL_Count)
{
    DisconParser parser;
    std::string filenameStr(filename);
    std::string priPathStr(priPath);

    if (!parser.load(filenameStr, ErrVar)) return;

    // ----------------------- Simulation Control --------------------------
    parser.parseInt("Echo",          CntrPar->Echo,          true);
    if (ErrVar->aviFAIL < 0) return;
    parser.parseInt("LoggingLevel",  CntrPar->LoggingLevel,  true);
    parser.parseDbl("DT_Out",        CntrPar->DT_Out,        true);
    parser.parseInt("Ext_Interface", CntrPar->Ext_Interface,  true);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Controller Flags --------------------------
    parser.parseInt("F_LPFType",       CntrPar->F_LPFType);
    parser.parseInt("IPC_ControlMode", CntrPar->IPC_ControlMode);
    parser.parseInt("VS_ControlMode",  CntrPar->VS_ControlMode);
    parser.parseInt("VS_ConstPower",   CntrPar->VS_ConstPower,   true);
    parser.parseInt("VS_FBP",          CntrPar->VS_FBP,          true);
    parser.parseInt("PC_ControlMode",  CntrPar->PC_ControlMode);
    parser.parseInt("Y_ControlMode",   CntrPar->Y_ControlMode);
    parser.parseInt("SS_Mode",         CntrPar->SS_Mode);
    parser.parseInt("PRC_Mode",        CntrPar->PRC_Mode);
    parser.parseInt("WE_Mode",         CntrPar->WE_Mode);
    parser.parseInt("PS_Mode",         CntrPar->PS_Mode);
    parser.parseInt("SU_Mode",         CntrPar->SU_Mode);
    parser.parseInt("SD_Mode",         CntrPar->SD_Mode);
    parser.parseInt("FL_Mode",         CntrPar->Fl_Mode);
    parser.parseInt("TD_Mode",         CntrPar->TD_Mode);
    parser.parseInt("TRA_Mode",        CntrPar->TRA_Mode);
    parser.parseInt("Flp_Mode",        CntrPar->Flp_Mode);
    parser.parseInt("OL_Mode",         CntrPar->OL_Mode);
    parser.parseInt("PA_Mode",         CntrPar->PA_Mode);
    parser.parseInt("PF_Mode",         CntrPar->PF_Mode);
    parser.parseInt("AWC_Mode",        CntrPar->AWC_Mode);
    parser.parseInt("Ext_Mode",        CntrPar->Ext_Mode);
    parser.parseInt("ZMQ_Mode",        CntrPar->ZMQ_Mode);
    parser.parseInt("CC_Mode",         CntrPar->CC_Mode);
    parser.parseInt("StC_Mode",        CntrPar->StC_Mode);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Filter Constants --------------------------
    parser.parseDbl("F_LPFCornerFreq",      CntrPar->F_LPFCornerFreq,      false);
    parser.parseDbl("F_LPFDamping",         CntrPar->F_LPFDamping,         CntrPar->F_LPFType == 1);
    parser.parseInt("F_NumNotchFilts",      CntrPar->F_NumNotchFilts,      true);
    // F_NotchFreqs, F_NotchBetaNum, F_NotchBetaDen arrays -> pass 2
    parser.parseInt("F_GenSpdNotch_N",      CntrPar->F_GenSpdNotch_N,      CntrPar->F_NumNotchFilts == 0);
    parser.parseInt("F_TwrTopNotch_N",      CntrPar->F_TwrTopNotch_N,      CntrPar->F_NumNotchFilts == 0);
    parser.parseDbl("F_SSCornerFreq",       CntrPar->F_SSCornerFreq,       CntrPar->SS_Mode == 0);
    parser.parseDbl("F_WECornerFreq",       CntrPar->F_WECornerFreq,       false);
    parser.parseDbl("F_YawErr",             CntrPar->F_YawErr,             CntrPar->Y_ControlMode == 0);
    // F_FlCornerFreq, F_FlpCornerFreq arrays -> pass 2
    parser.parseDbl("F_FlHighPassFreq",     CntrPar->F_FlHighPassFreq,     CntrPar->Fl_Mode == 0);
    parser.parseDbl("F_VSRefSpdCornerFreq", CntrPar->F_VSRefSpdCornerFreq, CntrPar->VS_ControlMode < 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Blade Pitch Control --------------------------
    parser.parseInt("PC_GS_n",      CntrPar->PC_GS_n,      CntrPar->PC_ControlMode == 0);
    // PC_GS_angles, KP, KI, KD, TF arrays -> pass 2
    parser.parseDbl("PC_MaxPit",    CntrPar->PC_MaxPit,    CntrPar->PC_ControlMode == 0);
    parser.parseDbl("PC_MinPit",    CntrPar->PC_MinPit,    CntrPar->PC_ControlMode == 0);
    parser.parseDbl("PC_MaxRat",    CntrPar->PC_MaxRat,    CntrPar->PC_ControlMode == 0);
    parser.parseDbl("PC_MinRat",    CntrPar->PC_MinRat,    CntrPar->PC_ControlMode == 0);
    parser.parseDbl("PC_RefSpd",    CntrPar->PC_RefSpd,    CntrPar->PC_ControlMode == 0);
    parser.parseDbl("PC_FinePit",   CntrPar->PC_FinePit,   CntrPar->PC_ControlMode == 0);
    parser.parseDbl("PC_Switch",    CntrPar->PC_Switch,    CntrPar->PC_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- IPC --------------------------
    // IPC_Vramp, KP, KI, aziOffset arrays -> pass 2
    parser.parseInt("IPC_SatMode",       CntrPar->IPC_SatMode,       CntrPar->IPC_ControlMode == 0);
    parser.parseDbl("IPC_IntSat",        CntrPar->IPC_IntSat,        CntrPar->IPC_ControlMode == 0);
    parser.parseDbl("IPC_CornerFreqAct", CntrPar->IPC_CornerFreqAct, CntrPar->IPC_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- VS Torque Control --------------------------
    parser.parseDbl("VS_GenEff",   CntrPar->VS_GenEff,   false);
    parser.parseDbl("VS_ArSatTq",  CntrPar->VS_ArSatTq,  CntrPar->VS_ControlMode != 1);
    parser.parseDbl("VS_MaxRat",   CntrPar->VS_MaxRat,   CntrPar->VS_ControlMode != 1);
    parser.parseDbl("VS_MaxTq",    CntrPar->VS_MaxTq,    false);
    parser.parseDbl("VS_MinTq",    CntrPar->VS_MinTq,    false);
    parser.parseDbl("VS_MinOMSpd", CntrPar->VS_MinOMSpd, true);
    parser.parseDbl("VS_Rgn2K",    CntrPar->VS_Rgn2K,    CntrPar->VS_ControlMode == 2);
    parser.parseDbl("VS_RtPwr",    CntrPar->VS_RtPwr,    false);
    parser.parseDbl("VS_RtTq",     CntrPar->VS_RtTq,     false);
    parser.parseDbl("VS_RefSpd",   CntrPar->VS_RefSpd,   false);
    parser.parseInt("VS_n",        CntrPar->VS_n,        false);
    // VS_KP, VS_KI arrays -> pass 2
    parser.parseDbl("VS_TSRopt",   CntrPar->VS_TSRopt,   CntrPar->VS_ControlMode < 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Fixed-Pitch Region 3 --------------------------
    parser.parseInt("VS_FBP_n", CntrPar->VS_FBP_n, CntrPar->VS_FBP == 0);
    // VS_FBP_U, Omega, Tau arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Setpoint Smoother --------------------------
    parser.parseDbl("SS_VSGain", CntrPar->SS_VSGain, CntrPar->SS_Mode == 0);
    parser.parseDbl("SS_PCGain", CntrPar->SS_PCGain, CntrPar->SS_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Power Reference --------------------------
    parser.parseInt("PRC_Comm",      CntrPar->PRC_Comm,      CntrPar->PRC_Mode != 1);
    parser.parseDbl("PRC_R_Torque",  CntrPar->PRC_R_Torque,  (CntrPar->PRC_Mode != 2) || (CntrPar->PRC_Comm != 0));
    parser.parseDbl("PRC_R_Speed",   CntrPar->PRC_R_Speed,   (CntrPar->PRC_Mode != 2) || (CntrPar->PRC_Comm != 0));
    parser.parseDbl("PRC_R_Pitch",   CntrPar->PRC_R_Pitch,   (CntrPar->PRC_Mode != 2) || (CntrPar->PRC_Comm != 0));
    parser.parseInt("PRC_Table_n",   CntrPar->PRC_Table_n,   (CntrPar->PRC_Mode != 2) || (CntrPar->PRC_R_Pitch == 1.0));
    // PRC_R_Table, PRC_Pitch_Table arrays -> pass 2
    parser.parseInt("PRC_n",         CntrPar->PRC_n,         CntrPar->PRC_Mode == 0);
    parser.parseDbl("PRC_LPF_Freq",  CntrPar->PRC_LPF_Freq,  CntrPar->PRC_Mode == 0);
    // PRC_WindSpeeds, PRC_GenSpeeds arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Wind Speed Estimator --------------------------
    parser.parseDbl("WE_BladeRadius",  CntrPar->WE_BladeRadius,  false);
    parser.parseDbl("WE_Gamma",        CntrPar->WE_Gamma,        CntrPar->WE_Mode != 1);
    parser.parseDbl("WE_GearboxRatio", CntrPar->WE_GearboxRatio, false);
    parser.parseDbl("WE_Jtot",         CntrPar->WE_Jtot,         CntrPar->WE_Mode == 0);
    parser.parseDbl("WE_RhoAir",       CntrPar->WE_RhoAir,       CntrPar->WE_Mode != 2);
    parser.parseStr("PerfFileName",    CntrPar->PerfFileName, 1024, CntrPar->WE_Mode == 0);
    // PerfTableSize array -> pass 2
    parser.parseInt("WE_FOPoles_N",    CntrPar->WE_FOPoles_N,    CntrPar->WE_Mode != 2);
    // WE_FOPoles_v, WE_FOPoles arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Yaw Control --------------------------
    parser.parseDbl("Y_uSwitch",    CntrPar->Y_uSwitch,    CntrPar->Y_ControlMode == 0);
    // Y_ErrThresh array -> pass 2
    parser.parseDbl("Y_Rate",       CntrPar->Y_Rate,       CntrPar->Y_ControlMode == 0);
    parser.parseDbl("Y_MErrSet",    CntrPar->Y_MErrSet,    CntrPar->Y_ControlMode == 0);
    parser.parseDbl("Y_IPC_IntSat", CntrPar->Y_IPC_IntSat, CntrPar->Y_ControlMode == 0);
    parser.parseDbl("Y_IPC_KP",     CntrPar->Y_IPC_KP,     CntrPar->Y_ControlMode == 0);
    parser.parseDbl("Y_IPC_KI",     CntrPar->Y_IPC_KI,     CntrPar->Y_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Tower Damper / TRA --------------------------
    parser.parseDbl("TRA_ExclSpeed",    CntrPar->TRA_ExclSpeed,    CntrPar->TRA_Mode == 0);
    parser.parseDbl("TRA_ExclBand",     CntrPar->TRA_ExclBand,     CntrPar->TRA_Mode == 0);
    parser.parseDbl("TRA_RateLimit",    CntrPar->TRA_RateLimit,    CntrPar->TRA_Mode == 0);
    parser.parseDbl("FA_KI",            CntrPar->FA_KI,            CntrPar->TD_Mode == 0);
    parser.parseDbl("FA_HPFCornerFreq", CntrPar->FA_HPFCornerFreq, CntrPar->TD_Mode == 0);
    parser.parseDbl("FA_IntSat",        CntrPar->FA_IntSat,        CntrPar->TD_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Peak Shaving --------------------------
    parser.parseInt("PS_BldPitchMin_N", CntrPar->PS_BldPitchMin_N, CntrPar->PS_Mode == 0);
    // PS_WindSpeeds, PS_BldPitchMin arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Startup --------------------------
    parser.parseDbl("SU_StartTime",            CntrPar->SU_StartTime,            CntrPar->SU_Mode == 0);
    parser.parseDbl("SU_FW_MinDuration",       CntrPar->SU_FW_MinDuration,       CntrPar->SU_Mode == 0);
    parser.parseDbl("SU_RotorSpeedThresh",     CntrPar->SU_RotorSpeedThresh,     CntrPar->SU_Mode == 0);
    parser.parseDbl("SU_RotorSpeedCornerFreq", CntrPar->SU_RotorSpeedCornerFreq, CntrPar->SU_Mode == 0);
    parser.parseInt("SU_LoadStages_N",         CntrPar->SU_LoadStages_N,         CntrPar->SU_Mode == 0);
    // SU_LoadStages, Ramp, Hold arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Shutdown --------------------------
    parser.parseDbl("SD_TimeActivate",       CntrPar->SD_TimeActivate,       CntrPar->SD_Mode == 0);
    parser.parseInt("SD_EnablePitch",        CntrPar->SD_EnablePitch,        CntrPar->SD_Mode == 0);
    parser.parseInt("SD_EnableYawError",     CntrPar->SD_EnableYawError,     CntrPar->SD_Mode == 0);
    parser.parseInt("SD_EnableGenSpeed",     CntrPar->SD_EnableGenSpeed,     CntrPar->SD_Mode == 0);
    parser.parseInt("SD_EnableTime",         CntrPar->SD_EnableTime,         CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_MaxPit",             CntrPar->SD_MaxPit,             CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_PitchCornerFreq",    CntrPar->SD_PitchCornerFreq,    CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_MaxYawError",        CntrPar->SD_MaxYawError,        CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_YawErrorCornerFreq", CntrPar->SD_YawErrorCornerFreq, CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_MaxGenSpd",          CntrPar->SD_MaxGenSpd,          CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_GenSpdCornerFreq",   CntrPar->SD_GenSpdCornerFreq,   CntrPar->SD_Mode == 0);
    parser.parseDbl("SD_Time",               CntrPar->SD_Time,               CntrPar->SD_Mode == 0);
    parser.parseInt("SD_Method",             CntrPar->SD_Method,             CntrPar->SD_Mode == 0);
    parser.parseInt("SD_Stage_N",            CntrPar->SD_Stage_N,            CntrPar->SD_Mode == 0);
    // SD_StageTime, StagePitch, MaxTorqueRate, MaxPitchRate arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Floating --------------------------
    parser.parseInt("Fl_n", CntrPar->Fl_n, true);
    if (CntrPar->Fl_n == 0) CntrPar->Fl_n = 1;  // Default is 1
    // Fl_Kp, Fl_U arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Flaps --------------------------
    parser.parseDbl("Flp_Angle",  CntrPar->Flp_Angle,  CntrPar->Flp_Mode == 0);
    parser.parseDbl("Flp_Kp",     CntrPar->Flp_Kp,     CntrPar->Flp_Mode == 0);
    parser.parseDbl("Flp_Ki",     CntrPar->Flp_Ki,     CntrPar->Flp_Mode == 0);
    parser.parseDbl("Flp_MaxPit", CntrPar->Flp_MaxPit, CntrPar->Flp_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Open Loop --------------------------
    parser.parseStr("OL_Filename",    CntrPar->OL_Filename, 1024, CntrPar->OL_Mode == 0);
    parser.parseInt("OL_BP_Mode",     CntrPar->OL_BP_Mode,        CntrPar->OL_Mode == 0);
    parser.parseDbl("OL_BP_FiltFreq", CntrPar->OL_BP_FiltFreq,    CntrPar->OL_Mode == 0);
    parser.parseInt("Ind_Breakpoint", CntrPar->Ind_Breakpoint,    true);
    // Ind_BldPitch array -> pass 2
    parser.parseInt("Ind_GenTq",      CntrPar->Ind_GenTq,         true);
    parser.parseInt("Ind_YawRate",    CntrPar->Ind_YawRate,       true);
    parser.parseInt("Ind_Azimuth",    CntrPar->Ind_Azimuth,       CntrPar->OL_Mode != 2);
    parser.parseInt("Ind_R_Speed",    CntrPar->Ind_R_Speed,       CntrPar->OL_Mode != 2);
    parser.parseInt("Ind_R_Torque",   CntrPar->Ind_R_Torque,      CntrPar->OL_Mode != 2);
    parser.parseInt("Ind_R_Pitch",    CntrPar->Ind_R_Pitch,       CntrPar->OL_Mode != 2);
    // RP_Gains array -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Pitch Actuator --------------------------
    parser.parseDbl("PA_CornerFreq", CntrPar->PA_CornerFreq, CntrPar->PA_Mode == 0);
    parser.parseDbl("PA_Damping",    CntrPar->PA_Damping,    CntrPar->PA_Mode == 0);
    // PF_Offsets, PF_TimeStuck arrays -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- AWC --------------------------
    parser.parseInt("AWC_NumModes",    CntrPar->AWC_NumModes,    CntrPar->AWC_Mode == 0);
    // AWC_n, harmonic, freq, amp, clockangle arrays -> pass 2
    parser.parseDbl("AWC_phaseoffset", CntrPar->AWC_phaseoffset, CntrPar->AWC_Mode == 0);
    // AWC_CntrGains array -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- External Control --------------------------
    parser.parseStr("DLL_FileName", CntrPar->DLL_FileName, 1024, CntrPar->Ext_Mode == 0);
    parser.parseStr("DLL_InFile",   CntrPar->DLL_InFile,   1024, CntrPar->Ext_Mode == 0);
    parser.parseStr("DLL_ProcName", CntrPar->DLL_ProcName, 1024, CntrPar->Ext_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- ZeroMQ --------------------------
    parser.parseInt("ZMQ_ID",           CntrPar->ZMQ_ID,           true);
    parser.parseStr("ZMQ_CommAddress",  CntrPar->ZMQ_CommAddress, 256, CntrPar->ZMQ_Mode == 0);
    parser.parseDbl("ZMQ_UpdatePeriod", CntrPar->ZMQ_UpdatePeriod, CntrPar->ZMQ_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Cable Control --------------------------
    parser.parseInt("CC_Group_N", CntrPar->CC_Group_N, CntrPar->CC_Mode == 0);
    // CC_GroupIndex array -> pass 2
    parser.parseDbl("CC_ActTau",  CntrPar->CC_ActTau,  CntrPar->CC_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Structural Control --------------------------
    parser.parseInt("StC_Group_N", CntrPar->StC_Group_N, CntrPar->StC_Mode == 0);
    // StC_GroupIndex array -> pass 2
    if (ErrVar->aviFAIL < 0) return;

    // ---------------------------------------------------------------
    // Count OL file rows if OL_Mode > 0
    // ---------------------------------------------------------------
    *n_OL_rows = 0;

    // Compute OL_Count for the Fortran wrapper
    int olCount = 0;
    if (CntrPar->OL_Mode > 0) {
        olCount = 1;  // breakpoint column

        // We need Ind_BldPitch for counting — parse it here as a temp
        // (arrays aren't allocated yet, so we parse from the file directly)
        int tmpIndBldPitch[3] = {0, 0, 0};
        {
            DisconParser tmpParser;
            tmpParser.load(filenameStr, ErrVar);
            if (ErrVar->aviFAIL < 0) return;
            tmpParser.parseIntAry("Ind_BldPitch", tmpIndBldPitch, 3, true);
        }

        if (tmpIndBldPitch[0] > 0) olCount++;
        if (tmpIndBldPitch[1] > 0) {
            // Don't increment if duplicate index
            if (!(tmpIndBldPitch[1] == tmpIndBldPitch[0] ||
                  tmpIndBldPitch[1] == tmpIndBldPitch[2]))
                olCount++;
        }
        if (tmpIndBldPitch[2] > 0) {
            if (!(tmpIndBldPitch[2] == tmpIndBldPitch[0] ||
                  tmpIndBldPitch[2] == tmpIndBldPitch[1]))
                olCount++;
        }
        if (CntrPar->Ind_GenTq > 0) olCount++;
        if (CntrPar->Ind_YawRate > 0) olCount++;
        if (CntrPar->Ind_Azimuth > 0 && CntrPar->OL_Mode == 2) olCount++;
        if (CntrPar->Ind_R_Speed > 0 && CntrPar->OL_Mode == 1) olCount++;
        if (CntrPar->Ind_R_Torque > 0 && CntrPar->OL_Mode == 1) olCount++;
        if (CntrPar->Ind_R_Pitch > 0 && CntrPar->OL_Mode == 1) olCount++;

        // Cable control indices (need CC_GroupIndex, not yet allocated — use CC_Group_N)
        // We parse Ind_CableControl and Ind_StructControl temporarily
        {
            DisconParser tmpParser;
            tmpParser.load(filenameStr, ErrVar);
            if (ErrVar->aviFAIL < 0) return;

            if (CntrPar->CC_Group_N > 0 && CntrPar->CC_Mode == 2) {
                std::vector<int> tmpIndCC(CntrPar->CC_Group_N, 0);
                tmpParser.parseIntAry("Ind_CableControl", tmpIndCC.data(), CntrPar->CC_Group_N, CntrPar->CC_Mode != 2);
                for (int i = 0; i < CntrPar->CC_Group_N; i++) {
                    if (tmpIndCC[i] > 0) olCount++;
                }
            }
            if (CntrPar->StC_Group_N > 0 && CntrPar->StC_Mode == 2) {
                std::vector<int> tmpIndStC(CntrPar->StC_Group_N, 0);
                tmpParser.parseIntAry("Ind_StructControl", tmpIndStC.data(), CntrPar->StC_Group_N, CntrPar->StC_Mode != 2);
                for (int i = 0; i < CntrPar->StC_Group_N; i++) {
                    if (tmpIndStC[i] > 0) olCount++;
                }
            }
        }

        // Resolve OL filename path
        std::string olFilename = trimFortranString(CntrPar->OL_Filename, 1024);
        if (!olFilename.empty() && pathIsRelative(olFilename)) {
            olFilename = priPathStr + olFilename;
        }

        // Count data rows in OL file
        std::ifstream olFile(olFilename);
        if (olFile.is_open()) {
            std::string line;
            int dataRows = 0;
            while (std::getline(olFile, line)) {
                // Skip empty lines
                if (line.empty()) continue;
                // Skip comment lines (starting with !, #, %)
                size_t first = line.find_first_not_of(" \t");
                if (first == std::string::npos) continue;
                char c = line[first];
                if (c == '!' || c == '#' || c == '%') continue;
                dataRows++;
            }
            *n_OL_rows = dataRows;
        }
        // If file doesn't exist, n_OL_rows stays 0; error will be caught in pass 2
    }

    *OL_Count = olCount;
}

// ---------------------------------------------------------------------------
// Pass 2 — fill all ALLOCATABLE array fields + computed constants + OL loading
// ---------------------------------------------------------------------------

void readcontrolparameterfilesub_pass2_c(
    controlparameters_view_t* CntrPar,
    localvariables_t* LocalVar,
    const char* filename,
    const char* priPath,
    errorvariables_t* ErrVar)
{
    DisconParser parser;
    std::string filenameStr(filename);
    std::string priPathStr(priPath);

    if (!parser.load(filenameStr, ErrVar)) return;

    // ----------------------- Filter Arrays --------------------------
    parser.parseDblAry("F_NotchFreqs",   CntrPar->F_NotchFreqs,   CntrPar->n_F_NotchFreqs,   CntrPar->F_NumNotchFilts == 0);
    parser.parseDblAry("F_NotchBetaNum", CntrPar->F_NotchBetaNum, CntrPar->n_F_NotchBetaNum, CntrPar->F_NumNotchFilts == 0);
    parser.parseDblAry("F_NotchBetaDen", CntrPar->F_NotchBetaDen, CntrPar->n_F_NotchBetaDen, CntrPar->F_NumNotchFilts == 0);
    if (CntrPar->F_GenSpdNotch_N > 0) {
        parser.parseIntAry("F_GenSpdNotch_Ind", CntrPar->F_GenSpdNotch_Ind, CntrPar->n_F_GenSpdNotch_Ind, CntrPar->F_NumNotchFilts == 0);
    }
    if (CntrPar->F_TwrTopNotch_N > 0) {
        parser.parseIntAry("F_TwrTopNotch_Ind", CntrPar->F_TwrTopNotch_Ind, CntrPar->n_F_TwrTopNotch_Ind, CntrPar->F_NumNotchFilts == 0);
    }
    parser.parseDblAry("F_FlCornerFreq",  CntrPar->F_FlCornerFreq,  2, CntrPar->Fl_Mode == 0);
    parser.parseDblAry("F_FlpCornerFreq", CntrPar->F_FlpCornerFreq, 2, CntrPar->Flp_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Blade Pitch Arrays --------------------------
    parser.parseDblAry("PC_GS_angles", CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles, CntrPar->PC_ControlMode == 0);
    parser.parseDblAry("PC_GS_KP",     CntrPar->PC_GS_KP,     CntrPar->n_PC_GS_KP,     CntrPar->PC_ControlMode == 0);
    parser.parseDblAry("PC_GS_KI",     CntrPar->PC_GS_KI,     CntrPar->n_PC_GS_KI,     CntrPar->PC_ControlMode == 0);
    parser.parseDblAry("PC_GS_KD",     CntrPar->PC_GS_KD,     CntrPar->n_PC_GS_KD,     CntrPar->PC_ControlMode == 0);
    parser.parseDblAry("PC_GS_TF",     CntrPar->PC_GS_TF,     CntrPar->n_PC_GS_TF,     CntrPar->PC_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- IPC Arrays --------------------------
    parser.parseDblAry("IPC_Vramp",     CntrPar->IPC_Vramp,     2, CntrPar->IPC_ControlMode == 0);
    parser.parseDblAry("IPC_KP",        CntrPar->IPC_KP,        2, CntrPar->IPC_ControlMode == 0);
    parser.parseDblAry("IPC_KI",        CntrPar->IPC_KI,        2, CntrPar->IPC_ControlMode == 0);
    parser.parseDblAry("IPC_aziOffset", CntrPar->IPC_aziOffset, 2, CntrPar->IPC_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- VS Arrays --------------------------
    parser.parseDblAry("VS_KP", CntrPar->VS_KP, CntrPar->n_VS_KP, false);
    parser.parseDblAry("VS_KI", CntrPar->VS_KI, CntrPar->n_VS_KI, false);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Fixed-Pitch Region 3 Arrays --------------------------
    parser.parseDblAry("VS_FBP_U",     CntrPar->VS_FBP_U,     CntrPar->n_VS_FBP_U,     CntrPar->VS_FBP == 0);
    parser.parseDblAry("VS_FBP_Omega", CntrPar->VS_FBP_Omega, CntrPar->n_VS_FBP_Omega, CntrPar->VS_FBP == 0);
    parser.parseDblAry("VS_FBP_Tau",   CntrPar->VS_FBP_Tau,   CntrPar->n_VS_FBP_Tau,   CntrPar->VS_FBP == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- PRC Arrays --------------------------
    parser.parseDblAry("PRC_R_Table",     CntrPar->PRC_R_Table,     CntrPar->n_PRC_R_Table,     (CntrPar->PRC_Mode != 2) || (CntrPar->PRC_R_Pitch == 1.0));
    parser.parseDblAry("PRC_Pitch_Table", CntrPar->PRC_Pitch_Table, CntrPar->n_PRC_Pitch_Table, (CntrPar->PRC_Mode != 2) || (CntrPar->PRC_R_Pitch == 1.0));
    parser.parseDblAry("PRC_WindSpeeds",  CntrPar->PRC_WindSpeeds,  CntrPar->n_PRC_WindSpeeds,  CntrPar->PRC_Mode == 0);
    parser.parseDblAry("PRC_GenSpeeds",   CntrPar->PRC_GenSpeeds,   CntrPar->n_PRC_GenSpeeds,   CntrPar->PRC_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Wind Speed Estimator Arrays --------------------------
    parser.parseIntAry("PerfTableSize", CntrPar->PerfTableSize, 2, CntrPar->WE_Mode == 0);
    parser.parseDblAry("WE_FOPoles_v",  CntrPar->WE_FOPoles_v, CntrPar->n_WE_FOPoles_v, CntrPar->WE_Mode != 2);
    parser.parseDblAry("WE_FOPoles",    CntrPar->WE_FOPoles,   CntrPar->n_WE_FOPoles,   CntrPar->WE_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Yaw Array --------------------------
    parser.parseDblAry("Y_ErrThresh", CntrPar->Y_ErrThresh, 2, CntrPar->Y_ControlMode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Peak Shaving Arrays --------------------------
    parser.parseDblAry("PS_WindSpeeds",  CntrPar->PS_WindSpeeds,  CntrPar->n_PS_WindSpeeds,  CntrPar->PS_Mode == 0);
    parser.parseDblAry("PS_BldPitchMin", CntrPar->PS_BldPitchMin, CntrPar->n_PS_BldPitchMin, CntrPar->PS_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Startup Arrays --------------------------
    parser.parseDblAry("SU_LoadStages",        CntrPar->SU_LoadStages,        CntrPar->n_SU_LoadStages,        CntrPar->SU_LoadStages_N == 0);
    parser.parseDblAry("SU_LoadRampDuration",  CntrPar->SU_LoadRampDuration,  CntrPar->n_SU_LoadRampDuration,  CntrPar->SU_LoadStages_N == 0);
    parser.parseDblAry("SU_LoadHoldDuration",  CntrPar->SU_LoadHoldDuration,  CntrPar->n_SU_LoadHoldDuration,  CntrPar->SU_LoadStages_N == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Shutdown Arrays --------------------------
    parser.parseDblAry("SD_StageTime",     CntrPar->SD_StageTime,     CntrPar->n_SD_StageTime,     CntrPar->SD_Method != 1);
    parser.parseDblAry("SD_StagePitch",    CntrPar->SD_StagePitch,    CntrPar->n_SD_StagePitch,    CntrPar->SD_Method != 2);
    parser.parseDblAry("SD_MaxTorqueRate", CntrPar->SD_MaxTorqueRate, CntrPar->n_SD_MaxTorqueRate, CntrPar->SD_Mode == 0);
    parser.parseDblAry("SD_MaxPitchRate",  CntrPar->SD_MaxPitchRate,  CntrPar->n_SD_MaxPitchRate,  CntrPar->SD_Mode == 0);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Floating Arrays --------------------------
    parser.parseDblAry("Fl_Kp", CntrPar->Fl_Kp, CntrPar->n_Fl_Kp, CntrPar->Fl_Mode == 0);
    parser.parseDblAry("Fl_U",  CntrPar->Fl_U,  CntrPar->n_Fl_U,  CntrPar->Fl_n == 1);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Open Loop Arrays --------------------------
    parser.parseIntAry("Ind_BldPitch", CntrPar->Ind_BldPitch, 3, true);
    parser.parseDblAry("RP_Gains",     CntrPar->RP_Gains,     4, CntrPar->OL_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Pitch Fault Arrays --------------------------
    parser.parseDblAry("PF_Offsets",   CntrPar->PF_Offsets,   3, CntrPar->PF_Mode != 1);
    parser.parseDblAry("PF_TimeStuck", CntrPar->PF_TimeStuck, 3, CntrPar->PF_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- AWC Arrays --------------------------
    parser.parseIntAry("AWC_n",          CntrPar->AWC_n,          CntrPar->n_AWC_n,          CntrPar->AWC_Mode != 1);
    parser.parseIntAry("AWC_harmonic",   CntrPar->AWC_harmonic,   CntrPar->n_AWC_harmonic,   CntrPar->AWC_Mode < 2);
    parser.parseDblAry("AWC_freq",       CntrPar->AWC_freq,       CntrPar->n_AWC_freq,       CntrPar->AWC_Mode == 0);
    parser.parseDblAry("AWC_amp",        CntrPar->AWC_amp,        CntrPar->n_AWC_amp,        CntrPar->AWC_Mode == 0);
    parser.parseDblAry("AWC_clockangle", CntrPar->AWC_clockangle, CntrPar->n_AWC_clockangle, CntrPar->AWC_Mode == 0);
    parser.parseDblAry("AWC_CntrGains",  CntrPar->AWC_CntrGains,  2,                         CntrPar->AWC_Mode < 3);
    if (ErrVar->aviFAIL < 0) return;

    // ----------------------- Cable Control Arrays --------------------------
    parser.parseIntAry("CC_GroupIndex",    CntrPar->CC_GroupIndex,    CntrPar->n_CC_GroupIndex,    CntrPar->CC_Mode == 0);
    parser.parseIntAry("StC_GroupIndex",   CntrPar->StC_GroupIndex,   CntrPar->n_StC_GroupIndex,   CntrPar->StC_Mode == 0);
    parser.parseIntAry("Ind_CableControl", CntrPar->Ind_CableControl, CntrPar->n_Ind_CableControl, CntrPar->CC_Mode != 2);
    parser.parseIntAry("Ind_StructControl", CntrPar->Ind_StructControl, CntrPar->n_Ind_StructControl, CntrPar->StC_Mode != 2);
    if (ErrVar->aviFAIL < 0) return;

    // ---------------------------------------------------------------
    // Computed Constants
    // ---------------------------------------------------------------

    // DT_Out default
    if (CntrPar->DT_Out == 0.0) CntrPar->DT_Out = LocalVar->DT;

    // Computed integers
    CntrPar->n_DT_Out = static_cast<int>(std::round(CntrPar->DT_Out / LocalVar->DT));
    CntrPar->n_DT_ZMQ = static_cast<int>(std::round(CntrPar->ZMQ_UpdatePeriod / LocalVar->DT));

    // Path resolution
    std::string perfFileName = trimFortranString(CntrPar->PerfFileName, 1024);
    if (!perfFileName.empty() && pathIsRelative(perfFileName)) {
        perfFileName = priPathStr + perfFileName;
        setFortranString(CntrPar->PerfFileName, 1024, perfFileName);
    }

    std::string olFilename = trimFortranString(CntrPar->OL_Filename, 1024);
    if (!olFilename.empty() && pathIsRelative(olFilename)) {
        olFilename = priPathStr + olFilename;
        setFortranString(CntrPar->OL_Filename, 1024, olFilename);
    }

    // Y_Rate conversion to deg/s
    CntrPar->Y_Rate = CntrPar->Y_Rate * R2D;

    // VS computed constants
    CntrPar->PC_RtTq99 = CntrPar->VS_RtTq * 0.99;
    CntrPar->VS_MinOMTq = CntrPar->VS_Rgn2K * CntrPar->VS_MinOMSpd * CntrPar->VS_MinOMSpd;
    CntrPar->VS_MaxOMTq = CntrPar->VS_Rgn2K * CntrPar->VS_RefSpd * CntrPar->VS_RefSpd;

    // ---------------------------------------------------------------
    // Open Loop CSV Loading (if OL_Mode > 0)
    // ---------------------------------------------------------------
    if (CntrPar->OL_Mode > 0) {
        // Build the OL channel display string and count
        std::string olString;
        int olCount = 1;  // breakpoint column

        if (CntrPar->Ind_BldPitch[0] > 0) {
            olString += " BldPitch1 ";
            olCount++;
        }
        if (CntrPar->Ind_BldPitch[1] > 0) {
            olString += " BldPitch2 ";
            if (!(CntrPar->Ind_BldPitch[1] == CntrPar->Ind_BldPitch[0] ||
                  CntrPar->Ind_BldPitch[1] == CntrPar->Ind_BldPitch[2])) {
                olCount++;
            }
        }
        if (CntrPar->Ind_BldPitch[2] > 0) {
            olString += " BldPitch3 ";
            if (!(CntrPar->Ind_BldPitch[2] == CntrPar->Ind_BldPitch[0] ||
                  CntrPar->Ind_BldPitch[2] == CntrPar->Ind_BldPitch[1])) {
                olCount++;
            }
        }
        if (CntrPar->Ind_GenTq > 0) {
            olString += " GenTq ";
            olCount++;
        }
        if (CntrPar->Ind_YawRate > 0) {
            olString += " YawRate ";
            olCount++;
        }
        if (CntrPar->Ind_Azimuth > 0 && CntrPar->OL_Mode == 2) {
            olString += " Azimuth ";
            olCount++;
        }
        if (CntrPar->Ind_R_Speed > 0 && CntrPar->OL_Mode == 1) {
            olString += " R_Speed ";
            olCount++;
        }
        if (CntrPar->Ind_R_Torque > 0 && CntrPar->OL_Mode == 1) {
            olString += " R_Torque ";
            olCount++;
        }
        if (CntrPar->Ind_R_Pitch > 0 && CntrPar->OL_Mode == 1) {
            olString += " R_Pitch ";
            olCount++;
        }

        int nOlCables = 0;
        if (CntrPar->n_Ind_CableControl > 0) {
            for (int i = 0; i < CntrPar->n_Ind_CableControl; i++) {
                if (CntrPar->Ind_CableControl[i] > 0) {
                    olString += " Cable" + std::to_string(i + 1) + " ";
                    olCount++;
                    nOlCables++;
                }
            }
        }

        int nOlStCs = 0;
        if (CntrPar->n_Ind_StructControl > 0) {
            for (int i = 0; i < CntrPar->n_Ind_StructControl; i++) {
                if (CntrPar->Ind_StructControl[i] > 0) {
                    olString += " StC" + std::to_string(i + 1) + " ";
                    olCount++;
                    nOlStCs++;
                }
            }
        }

        std::printf(" ROSCO: Implementing open loop control for%s\n", olString.c_str());
        if (CntrPar->OL_Mode == 2) {
            std::printf(" ROSCO: OL_Mode = 2 will change generator torque control for Azimuth tracking\n");
        }

        // Resolve OL filename (already resolved above in computed constants)
        std::string olPath = trimFortranString(CntrPar->OL_Filename, 1024);

        // Read OL file: Read_OL_Input equivalent
        std::ifstream olFile(olPath);
        if (!olFile.is_open()) {
            std::string msg = olPath + " does not exist";
            setError(ErrVar, msg.c_str());
            return;
        }

        // Read all lines, separate comments from data
        std::vector<std::string> dataLines;
        {
            std::string line;
            while (std::getline(olFile, line)) {
                if (line.empty()) continue;
                size_t first = line.find_first_not_of(" \t");
                if (first == std::string::npos) continue;
                char c = line[first];
                if (c == '!' || c == '#' || c == '%') continue;
                dataLines.push_back(line);
            }
        }

        int nRows = static_cast<int>(dataLines.size());
        if (nRows < 1) {
            setError(ErrVar, "Error: No data lines found in OL input file.");
            return;
        }

        // Parse all data into OL_Channels (column-major: col * nRows + row)
        // OL_Channels is already allocated by Fortran: n_OL_Channels_rows x n_OL_Channels_cols
        int nCols = CntrPar->n_OL_Channels_cols;
        for (int i = 0; i < nRows; i++) {
            std::istringstream iss(dataLines[i]);
            for (int j = 0; j < nCols; j++) {
                double val = 0.0;
                iss >> val;
                // Column-major: OL_Channels[col * nRows + row]
                CntrPar->OL_Channels[j * nRows + i] = val;
            }
        }

        // Extract columns into individual arrays based on Ind_* fields
        // Ind_* are 1-based Fortran column indices
        // OL_Breakpoints
        if (CntrPar->OL_Breakpoints && CntrPar->Ind_Breakpoint > 0) {
            int col = CntrPar->Ind_Breakpoint - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_Breakpoints[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_BldPitch1
        if (CntrPar->OL_BldPitch1 && CntrPar->Ind_BldPitch[0] > 0) {
            int col = CntrPar->Ind_BldPitch[0] - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_BldPitch1[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_BldPitch2
        if (CntrPar->OL_BldPitch2 && CntrPar->Ind_BldPitch[1] > 0) {
            int col = CntrPar->Ind_BldPitch[1] - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_BldPitch2[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_BldPitch3
        if (CntrPar->OL_BldPitch3 && CntrPar->Ind_BldPitch[2] > 0) {
            int col = CntrPar->Ind_BldPitch[2] - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_BldPitch3[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_GenTq
        if (CntrPar->OL_GenTq && CntrPar->Ind_GenTq > 0) {
            int col = CntrPar->Ind_GenTq - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_GenTq[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_YawRate
        if (CntrPar->OL_YawRate && CntrPar->Ind_YawRate > 0) {
            int col = CntrPar->Ind_YawRate - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_YawRate[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_Azimuth (with unwrap)
        if (CntrPar->OL_Azimuth && CntrPar->Ind_Azimuth > 0) {
            int col = CntrPar->Ind_Azimuth - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_Azimuth[i] = CntrPar->OL_Channels[col * nRows + i];
            }
            unwrap(CntrPar->OL_Azimuth, nRows);
        }

        // OL_R_Speed
        if (CntrPar->OL_R_Speed && CntrPar->Ind_R_Speed > 0) {
            int col = CntrPar->Ind_R_Speed - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_R_Speed[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_R_Torque
        if (CntrPar->OL_R_Torque && CntrPar->Ind_R_Torque > 0) {
            int col = CntrPar->Ind_R_Torque - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_R_Torque[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_R_Pitch
        if (CntrPar->OL_R_Pitch && CntrPar->Ind_R_Pitch > 0) {
            int col = CntrPar->Ind_R_Pitch - 1;
            for (int i = 0; i < nRows; i++) {
                CntrPar->OL_R_Pitch[i] = CntrPar->OL_Channels[col * nRows + i];
            }
        }

        // OL_CableControl (2D, column-major: col * nRows + row)
        if (nOlCables > 0 && CntrPar->OL_CableControl) {
            int iOL = 0;
            for (int i = 0; i < CntrPar->n_Ind_CableControl; i++) {
                if (CntrPar->Ind_CableControl[i] > 0) {
                    int col = CntrPar->Ind_CableControl[i] - 1;
                    for (int r = 0; r < nRows; r++) {
                        // OL_CableControl is (nOlCables, nRows) in Fortran
                        // Column-major: element(iOL, r) = OL_CableControl[r * nOlCables + iOL]
                        CntrPar->OL_CableControl[r * nOlCables + iOL] = CntrPar->OL_Channels[col * nRows + r];
                    }
                    iOL++;
                }
            }
        }

        // OL_StructControl (2D, column-major: col * nRows + row)
        if (nOlStCs > 0 && CntrPar->OL_StructControl) {
            int iOL = 0;
            for (int i = 0; i < CntrPar->n_Ind_StructControl; i++) {
                if (CntrPar->Ind_StructControl[i] > 0) {
                    int col = CntrPar->Ind_StructControl[i] - 1;
                    for (int r = 0; r < nRows; r++) {
                        CntrPar->OL_StructControl[r * nOlStCs + iOL] = CntrPar->OL_Channels[col * nRows + r];
                    }
                    iOL++;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Housekeeping — add RoutineName to error message
    // ---------------------------------------------------------------
    if (ErrVar->aviFAIL < 0) {
        std::string current = trimFortranString(ErrVar->ErrMsg, 1024);
        std::string prefixed = "ReadControlParameterFileSub:" + current;
        setFortranString(ErrVar->ErrMsg, 1024, prefixed);
    }
}

} // extern "C"
