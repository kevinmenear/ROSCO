// VIT Translation Scaffold
// Function: unwrap
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION unwrap(x, ErrVar) RESULT(y)
// Source MD5: 0b1a2a46739c
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-04-02T21:25:20Z

#include "vit_types.h"
#include "rosco_constants.h"
#include <cstring>
#include <cstdio>

void unwrap(double* x, int n_x, errorvariables_t* ErrVar, double* unwrap_result) {
    // Copy input to result (Fortran: y = x)
    for (int i = 0; i < n_x; i++) {
        unwrap_result[i] = x[i];
    }

    // Unwrap: adjust elements from i onward by ±2*PI until consecutive
    // differences are in [-PI, PI]
    // Fortran is 1-based (i = 2 to SIZE(x)), C++ is 0-based (i = 1 to n_x-1)
    for (int i = 1; i < n_x; i++) {
        while (unwrap_result[i] - unwrap_result[i - 1] <= -PI) {
            for (int j = i; j < n_x; j++) {
                unwrap_result[j] += 2.0 * PI;
            }
        }
        while (unwrap_result[i] - unwrap_result[i - 1] >= PI) {
            for (int j = i; j < n_x; j++) {
                unwrap_result[j] -= 2.0 * PI;
            }
        }
    }

    // Prepend routine name to error message if aviFAIL < 0
    if (ErrVar->aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "unwrap:%s", ErrVar->ErrMsg);
        // Pad with spaces to match Fortran CHARACTER(1024) semantics
        int len = (int)strlen(tmp);
        memcpy(ErrVar->ErrMsg, tmp, len);
        for (int k = len; k < 1024; k++) {
            ErrVar->ErrMsg[k] = ' ';
        }
    }
}
