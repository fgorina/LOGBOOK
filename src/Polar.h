#pragma once

// Polar efficiency = SOG / interpolated target boat speed(TWA, TWS).
//
//   twaDeg : true wind angle off the bow, 0-180 (deg)
//   twsKn  : true wind speed (knots)
//   sogKn  : speed over ground (knots)
//
// TWS is clamped to the table range (6-20 kn). Returns 0 when TWA is above the
// pointing range (below the interpolated beat angle for that TWS) or when the
// interpolated target speed is not positive.
double polarEfficiency(double twaDeg, double twsKn, double sogKn);
