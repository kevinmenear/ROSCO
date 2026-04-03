#include "../include/vit_types.h"
#include <cstring>
#include <cstdio>

double interp1d(double* xData, int n_xData, double* yData, int n_yData, double xq, errorvariables_t* ErrVar) {
    // Error check: are xData and yData the same size?
    if (n_xData != n_yData) {
        ErrVar->aviFAIL = -1;
        int len = snprintf(ErrVar->ErrMsg, 1024,
            " SIZE(xData) =%2d and SIZE(yData) =%2d are not the same",
            n_xData, n_yData);
        if (len >= 0 && len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
    }

    // Error check: xData must be strictly increasing
    for (int i = 0; i < n_xData - 1; i++) {
        if (xData[i + 1] - xData[i] <= 0.0) {
            ErrVar->aviFAIL = -1;
            const char msg[] = " xData is not strictly increasing";
            int len = (int)sizeof(msg) - 1;
            memcpy(ErrVar->ErrMsg, msg, len);
            if (len < 1024) memset(ErrVar->ErrMsg + len, ' ', 1024 - len);
            break;
        }
    }

    // Interpolate
    double result;
    int n = n_xData;
    if (xq <= xData[0]) {
        result = yData[0];
    } else if (xq >= xData[n - 1]) {
        result = yData[n - 1];
    } else {
        result = yData[n - 1]; // fallback (should not reach)
        for (int i = 1; i < n; i++) {
            if (xq <= xData[i]) {
                result = yData[i - 1] + (yData[i] - yData[i - 1]) / (xData[i] - xData[i - 1]) * (xq - xData[i - 1]);
                break;
            }
        }
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // Fortran: ErrVar%ErrMsg = 'interp1d'//':'//TRIM(ErrVar%ErrMsg)
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') {
            trimmed_len--;
        }
        char buf[1024];
        const char prefix[] = "interp1d:";
        int prefix_len = 9;
        memcpy(buf, prefix, prefix_len);
        int copy_len = trimmed_len;
        if (prefix_len + copy_len > 1024) copy_len = 1024 - prefix_len;
        memcpy(buf + prefix_len, ErrVar->ErrMsg, copy_len);
        int total = prefix_len + copy_len;
        if (total < 1024) memset(buf + total, ' ', 1024 - total);
        memcpy(ErrVar->ErrMsg, buf, 1024);
    }

    return result;
}
