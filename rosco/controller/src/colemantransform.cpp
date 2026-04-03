#include <cmath>
#include "rosco_constants.h"

void ColemanTransform(double* rootMOOP, double aziAngle, int nHarmonic, double* axTOut, double* axYOut) {
    const double phi2 = 2.0 / 3.0 * PI;
    const double phi3 = 4.0 / 3.0 * PI;

    *axTOut = 2.0 / 3.0 * (cos(nHarmonic * aziAngle) * rootMOOP[0]
            + cos(nHarmonic * (aziAngle + phi2)) * rootMOOP[1]
            + cos(nHarmonic * (aziAngle + phi3)) * rootMOOP[2]);
    *axYOut = 2.0 / 3.0 * (sin(nHarmonic * aziAngle) * rootMOOP[0]
            + sin(nHarmonic * (aziAngle + phi2)) * rootMOOP[1]
            + sin(nHarmonic * (aziAngle + phi3)) * rootMOOP[2]);
}
