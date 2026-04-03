// VIT Translation
// Function: ReadRestartFile
// Source: ROSCO_IO.f90
// Module: ROSCO_IO
// Status: unverified

#include "restart_fields.h"

void ReadRestartFile(float* avrSWAP, localvariables_t* LocalVar,
                     controlparameters_view_t* /*CntrPar*/, objectinstances_t* objInst,
                     performancedata_view_t* /*PerfData*/, char* RootName,
                     int size_avcOUTNAME, errorvariables_t* ErrVar) {
    std::string root = trim_fortran_string(RootName, size_avcOUTNAME);
    // Fortran: NINT(avrSWAP(2)/avrSWAP(3))  — 1-indexed
    int timestep = (int)std::round((double)avrSWAP[1] / (double)avrSWAP[2]);
    std::string filename = root + std::to_string(timestep) + ".RO.chkp";

    std::ifstream f(filename, std::ios::binary);
    if (!f.is_open()) {
        ErrVar->aviFAIL = 1;
        snprintf(ErrVar->ErrMsg, sizeof(ErrVar->ErrMsg),
                 "ROSCO_IO: Cannot open checkpoint file %s for reading", filename.c_str());
        return;
    }

    checkpoint_fields(f, LocalVar, objInst, [](std::ifstream& s, auto& val) {
        read_field(s, val);
    });

    if (!f.good()) {
        ErrVar->aviFAIL = 1;
        snprintf(ErrVar->ErrMsg, sizeof(ErrVar->ErrMsg),
                 "ROSCO_IO: Error reading checkpoint file.");
    }

    // Note: ReadControlParameterFileSub and ReadCpFile calls are handled
    // by the Fortran wrapper, not here.
}

// extern "C" wrapper — manual integration (VIT doesn't generate this for ReadRestartFile
// because it needs callee dispatch in the Fortran wrapper)
extern "C" {
    void readrestartfile_c(float* avrSWAP, localvariables_t* LocalVar,
                           controlparameters_view_t* CntrPar, objectinstances_t* objInst,
                           performancedata_view_t* PerfData, char* RootName,
                           int size_avcOUTNAME, errorvariables_t* ErrVar) {
        ReadRestartFile(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName,
                        size_avcOUTNAME, ErrVar);
    }
}
