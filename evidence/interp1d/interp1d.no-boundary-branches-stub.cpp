// KERNEL PROBE — INPUT to a measurement, not the measurement.
//
// The shipped translation with the two BOUNDARY branches deleted: only the
// interior `DO I = 1, SIZE(xData)` search remains. It answers the question the
// verdict line cannot -- how many of the 62 captured cases take
// `xq <= MINVAL(xData)` or `xq >= MAXVAL(xData)` at all.
//
// If this passes 62 of 62, no captured case reaches either endpoint branch and
// the kernel constrains one third of this unit's control flow. Everything else
// -- the error branches, the RoutineName prefix, the arguments -- is unchanged
// from the shipped file, so a moved row can only be an endpoint case.
//
//     cp this kernel/interp1d/interp1d.hpp
//     cd kernel/interp1d && rm -f interp1d.o kernel.exe && make -s build && ./kernel.exe

#include "vit_types.h"

#include <cfloat>
#include <cstdio>
#include <cstring>

namespace {

constexpr char RoutineName[] = "interp1d";
constexpr char MsgSizeMismatch[] = " xData and yData are not the same size";
constexpr char MsgNotIncreasing[] = " xData is not strictly increasing";

void assign_errmsg(errorvariables_view_t* ErrVar, const char* s, int n) {
    if (ErrVar->ErrMsg == nullptr) return;
    if (n > static_cast<int>(ErrVar->n_ErrMsg_cap)) return;
    std::memcpy(ErrVar->ErrMsg, s, static_cast<size_t>(n));
    ErrVar->n_ErrMsg = static_cast<int32_t>(n);
}

int len_trim(const char* s, int n) {
    while (n > 0 && s[n - 1] == ' ') --n;
    return n;
}

}  // namespace

double interp1d(double* xData, int n_xData, double* yData, int n_yData, double xq,
                errorvariables_view_t* ErrVar) {
    double interp1d_result = 0.0;

    if (n_xData != n_yData) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, MsgSizeMismatch, sizeof(MsgSizeMismatch) - 1);
    }

    for (int I_DIFF = 1; I_DIFF <= n_xData - 1; ++I_DIFF) {
        if (xData[I_DIFF] - xData[I_DIFF - 1] <= 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, MsgNotIncreasing, sizeof(MsgNotIncreasing) - 1);
            break;
        }
    }

    // THE TWO BOUNDARY BRANCHES ARE GONE. Only the interior search remains.
    for (int I = 1; I <= n_xData; ++I) {
        if (xq <= xData[I - 1]) {
            interp1d_result = yData[I - 2] +
                              (yData[I - 1] - yData[I - 2]) /
                                  (xData[I - 1] - xData[I - 2]) *
                                  (xq - xData[I - 2]);
            break;
        }
    }

    if (ErrVar->aviFAIL < 0) {
        const int len_RoutineName = sizeof(RoutineName) - 1;
        if (ErrVar->ErrMsg != nullptr && ErrVar->n_ErrMsg > 0) {
            const int lt = len_trim(ErrVar->ErrMsg, static_cast<int>(ErrVar->n_ErrMsg));
            const int len_new = len_RoutineName + 1 + lt;
            if (len_new <= static_cast<int>(ErrVar->n_ErrMsg_cap)) {
                std::memmove(ErrVar->ErrMsg + len_RoutineName + 1, ErrVar->ErrMsg,
                             static_cast<size_t>(lt));
                std::memcpy(ErrVar->ErrMsg, RoutineName,
                            static_cast<size_t>(len_RoutineName));
                ErrVar->ErrMsg[len_RoutineName] = ':';
                ErrVar->n_ErrMsg = static_cast<int32_t>(len_new);
            }
        }
    }

    return interp1d_result;
}
