/* A minimal Bladed-style external controller, for use as ExtController's ORACLE.
 *
 * WHY THIS EXISTS.
 *
 * `ExtController` loads an external dynamic library named by `CntrPar%DLL_FileName`
 * and calls the procedure named by `CntrPar%DLL_ProcName` through a
 * `C_F_PROCPOINTER`. It does NOT check `ErrVar%ErrStat` after `LoadDynamicLib`,
 * so when `dlopen` fails the procedure address stays `C_NULL_FUNPTR` and the
 * CALL two statements later dereferences it: SIGSEGV, measured at unit #5 and
 * recorded in `evidence/ExtController/probe_ext_mode_1.json`.
 *
 * `DLL_FileName` is the literal string `"unused"` in all 14 Examples inputs, and
 * before this file NO external Bladed-style library existed anywhere in the tree.
 * So the ORIGINAL FORTRAN could not be executed to completion on any input this
 * campaign possessed -- which is what made unit #5 `blocked`, since P7 makes the
 * original the oracle for every verification layer.
 *
 * This library is the missing fixture. It is an ADDITION (P5): nothing already
 * in the tree changes, no gate scenario is added, no baseline moves, and the
 * gate's 27 scenarios keep `Ext_Mode = 0` and never load it. It exists so that
 * the DIFFERENTIAL HARNESS can call the clean Fortran `ExtController` and get an
 * answer back instead of a signal.
 *
 * WHAT IT MUST BE, and why each property is load-bearing:
 *
 *   1. DETERMINISTIC. The harness calls the Fortran reference and the C++
 *      translation with the same inputs and compares outputs bit-for-bit. Both
 *      sides call THIS function. Anything non-deterministic here (a clock, an
 *      address, uninitialised memory) would present as a translation defect.
 *
 *   2. DEPENDENT ON ITS INPUTS. A stub that ignores `avrSWAP` and writes
 *      constants would let a translation that never fills `ExtDLL%avrSWAP` pass
 *      -- the same vacuity as unit #2's all-zero kernel window. So the values it
 *      writes back are a function of the values it was given, and of the three
 *      length records `ExtController` itself computes (49, 50, 51), which is the
 *      only arithmetic the unit does.
 *
 *   3. NO STATE. It is called once per harness case, possibly many times per
 *      process, and must answer the same way each time.
 *
 * SIGNATURE. From `ExtControl.f90`'s own ABSTRACT INTERFACE
 * `BladedDLL_Legacy_Procedure`, which is BIND(C):
 *
 *     REAL(ReKi)             INTENT(INOUT) :: avrSWAP(*)     -> float*
 *     INTEGER(C_INT)         INTENT(INOUT) :: aviFAIL        -> int*
 *     CHARACTER(KIND=C_CHAR) INTENT(IN)    :: accINFILE(*)   -> char*
 *     CHARACTER(KIND=C_CHAR) INTENT(INOUT) :: avcOUTNAME(*)  -> char*
 *     CHARACTER(KIND=C_CHAR) INTENT(INOUT) :: avcMSG(*)      -> char*
 *
 * `ReKi` is `C_FLOAT` (Constants.f90:18), so `avrSWAP` is `float*` and NOT
 * `double*` despite the project's `-fdefault-real-8`: an explicit kind is not
 * promoted by that flag. Getting this wrong is silent -- the array is assumed
 * size, so nothing on either side states how many bytes it is.
 *
 * The exported name is `DISCON`, because `DLL_ProcName` is `"DISCON"` in all 14
 * inputs and `LoadDynamicLibProc` looks up exactly that string with `dlsym`.
 *
 * Build (see build.sh):
 *     gcc -shared -fPIC -O2 -o libdiscon_stub.so discon_stub.c
 */

#include <string.h>

/* The records ExtController writes before the call. Named rather than spelled as
 * numbers at the use site, so a reader can check them against the Fortran. */
#define REC_MSG_LEN      49   /* LEN(avcMSG)   + 1 */
#define REC_INFILE_LEN   50   /* LEN_TRIM(DLL_InFile) + 1 */
#define REC_OUTNAME_LEN  51   /* LEN_TRIM(ExtRootName) + 1 */

/* Records the stub answers in. Chosen in the Bladed range ExtController's own
 * comment calls "data from external dll", and deliberately NOT 49/50/51, so the
 * inputs the unit computes and the outputs the stub produces are distinct
 * storage: a translation that wrote record 49 into record 45 would be caught. */
#define REC_OUT_A        45
#define REC_OUT_B        47
#define REC_OUT_C        48

/* Fortran is 1-based; C is 0-based. One place, so the off-by-one is stated
 * once rather than repeated at six use sites. */
#define R(i) ((i) - 1)

void DISCON(float *avrSWAP, int *aviFAIL, const char *accINFILE,
            char *avcOUTNAME, char *avcMSG)
{
    /* A deterministic, input-dependent answer.
     *
     * Each output reads a DIFFERENT input, so a translation that transposed two
     * records, or that filled ExtDLL%avrSWAP from the wrong source, changes at
     * least one of them. Integer-valued float arithmetic on purpose: the three
     * length records are small integers, so these are exact in binary32 and the
     * comparison stays bit-for-bit without a tolerance. */
    avrSWAP[R(REC_OUT_A)] = avrSWAP[R(REC_MSG_LEN)] * 2.0f
                          + avrSWAP[R(REC_INFILE_LEN)];
    avrSWAP[R(REC_OUT_B)] = avrSWAP[R(REC_OUTNAME_LEN)] * 3.0f
                          - avrSWAP[R(REC_MSG_LEN)];
    /* Record 1 is the Bladed status flag and record 2 the simulation time; both
     * come straight from the caller's own avrSWAP, so this ties the stub's
     * answer to the array the unit was asked to copy in. */
    avrSWAP[R(REC_OUT_C)] = avrSWAP[R(1)] + avrSWAP[R(2)];

    /* aviFAIL is INTENT(INOUT) in the interface and ExtController zeroes it
     * before the call. 0 means "successful, no message". */
    *aviFAIL = 0;

    /* avcMSG is INTENT(INOUT) and ExtController hands it a buffer of
     * LEN(ErrVar%ErrMsg)+1 bytes, initialised to a single NUL. Writing a NUL is
     * the "no message" answer and cannot overrun a buffer of any length >= 1. */
    avcMSG[0] = '\0';

    /* accINFILE and avcOUTNAME are read, not answered. Touching them keeps a
     * compiler from concluding the parameters are unused, and reading them is
     * what the real Bladed contract does. */
    (void)accINFILE;
    (void)avcOUTNAME;
}
