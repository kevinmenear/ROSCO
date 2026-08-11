#ifndef ERRORVARIABLES_VIEW_T_H
#define ERRORVARIABLES_VIEW_T_H

#include <stdint.h>

#ifndef VIT_COMPLEX_T_DEFINED
#define VIT_COMPLEX_T_DEFINED
typedef struct { double re, im; } vit_complex_double;
typedef struct { float  re, im; } vit_complex_float;
#endif

typedef struct {
    // --- Scalar and fixed-size fields ---
    int size_avcMSG;
    int aviFAIL;
    int ErrStat;

    // --- ALLOCATABLE fields (pointer + size) ---
    char* ErrMsg;  // deferred-length CHARACTER — blank-padded staging buffer
    int32_t n_ErrMsg;      // current LEN; set to the new LEN to reassign
    int32_t n_ErrMsg_cap;  // bytes of ErrMsg the C++ may write
} errorvariables_view_t;

#endif // ERRORVARIABLES_VIEW_T_H
