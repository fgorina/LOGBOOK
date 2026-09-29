#include "Polar.h"
#include <math.h>

namespace {

// yamato_polar_final.csv — target boat speed (knots) at TWA (rows) x TWS (cols)
const int N_TWA = 8;
const int N_TWS = 7;
const double TWA_GRID[N_TWA] = {52, 60, 75, 90, 110, 120, 135, 150};
const double TWS_GRID[N_TWS] = {6, 8, 10, 12, 14, 16, 20};
const double BSP[N_TWA][N_TWS] = {
    {3.28, 3.94, 4.43, 4.70, 4.74, 4.78, 4.83},
    {3.37, 4.10, 4.56, 4.79, 4.88, 4.92, 4.97},
    {3.76, 4.57, 5.02, 5.27, 5.37, 5.57, 5.82},
    {4.48, 5.44, 5.98, 6.22, 6.40, 6.58, 6.82},
    {4.21, 5.05, 5.61, 5.89, 6.00, 6.23, 6.68},
    {3.77, 4.78, 5.29, 5.57, 5.77, 6.08, 6.50},
    {3.12, 4.37, 4.80, 5.09, 5.42, 5.86, 6.24},
    {3.00, 4.20, 4.62, 5.04, 5.22, 5.68, 6.33},
};

// yamato_polar_beat_run.csv — optimal beat angle (deg) by TWS
const double BEAT_ANGLE[N_TWS] = {48.6, 46.8, 45.0, 44.1, 43.6, 43.2, 43.2};

double bilinear(double dA, double dD,
                double bsp00, double bsp10, double bsp01, double bsp11) {
    double w00 = (1 - dA) * (1 - dD);
    double w10 = dA * (1 - dD);
    double w01 = (1 - dA) * dD;
    double w11 = dA * dD;
    return w00 * bsp00 + w10 * bsp10 + w01 * bsp01 + w11 * bsp11;
}

double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Index i with grid[i] <= x <= grid[i+1] and the fraction toward grid[i+1].
// x is assumed already clamped to [grid[0], grid[n-1]].
void bracket(const double *grid, int n, double x, int &i, double &frac) {
    i = 0;
    while (i < n - 2 && x > grid[i + 1]) i++;
    frac = (x - grid[i]) / (grid[i + 1] - grid[i]);
}

double interp1(const double *grid, const double *val, int n, double x) {
    if (x <= grid[0]) return val[0];
    if (x >= grid[n - 1]) return val[n - 1];
    int i; double f;
    bracket(grid, n, x, i, f);
    return val[i] * (1 - f) + val[i + 1] * f;
}

} // namespace

double polarEfficiency(double twaDeg, double twsKn, double sogKn) {
    twsKn = clampd(twsKn, TWS_GRID[0], TWS_GRID[N_TWS - 1]);

    if (twaDeg < interp1(TWS_GRID, BEAT_ANGLE, N_TWS, twsKn)) return 0.0;

    twaDeg = clampd(twaDeg, TWA_GRID[0], TWA_GRID[N_TWA - 1]);

    int ia, id;
    double dA, dD;
    bracket(TWA_GRID, N_TWA, twaDeg, ia, dA);
    bracket(TWS_GRID, N_TWS, twsKn, id, dD);

    double bsp = bilinear(dA, dD,
                          BSP[ia][id],     BSP[ia + 1][id],
                          BSP[ia][id + 1], BSP[ia + 1][id + 1]);
    if (bsp <= 0.0) return 0.0;
    return sogKn / bsp;
}
