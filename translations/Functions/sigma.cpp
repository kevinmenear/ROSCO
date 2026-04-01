// VIT Translation Scaffold
// Function: sigma
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION sigma(x, x0, x1, y0, y1, ErrVar)
// Source MD5: 551e04b47f99
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-03-19T00:10:51Z
// Generic sigma function — smooth cubic transition
// Returns y0 when x < x0, y1 when x > x1, cubic blend in between
// Chains error message if aviFAIL < 0 on entry

#include <cstring>

#include "errorvariables_t.h"

double sigma(double x, double x0, double x1, double y0, double y1, errorvariables_t* ErrVar) {
    double d = x0 - x1;
    double d3 = d * d * d;
    double a3 = 2.0 / d3;
    double a2 = -3.0 * (x0 + x1) / d3;
    double a1 = 6.0 * x1 * x0 / d3;
    double a0 = (x0 - 3.0 * x1) * x0 * x0 / d3;

    double result;
    if (x < x0) {
        result = y0;
    } else if (x > x1) {
        result = y1;
    } else {
        result = (a3 * x * x * x + a2 * x * x + a1 * x + a0) * (y1 - y0) + y0;
    }

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // Fortran: ErrVar%ErrMsg = 'sigma'//':'//TRIM(ErrVar%ErrMsg)
        int trimmed_len = 1024;
        while (trimmed_len > 0 && ErrVar->ErrMsg[trimmed_len - 1] == ' ') {
            trimmed_len--;
        }
        char buf[1024];
        const char prefix[] = "sigma:";
        int prefix_len = 6;
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
