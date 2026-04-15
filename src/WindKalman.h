#pragma once
#include <cmath>
#include <cstdint>

// Result of one WindKalmanPolar update
struct WindKalmanResult {
    float angle;      // filtered TWD (radians, nautical: 0 = N, clockwise)
    float speed;      // filtered TWS (m/s)
    float angleRate;  // rad/s
    float speedRate;  // m/s²
};

// Standalone 2D polar Kalman filter for TWD / TWS.
//
// Converts (angle, speed) → Cartesian wind vector, runs a 4-state linear
// Kalman [ vx, vy, vx', vy' ], then converts back to polar via Jacobian.
//
// Tuning matches Tab5Nav TWDTWS_filter:
//   q = 0.01   qRate = 0.005   r = 0.5
class WindKalmanPolar {
public:
    WindKalmanPolar();

    // Feed one raw sample.
    // nowMs: millisecond timestamp at the moment the measurement was received
    //        (pass millis() from the call site, not from inside the filter).
    WindKalmanResult update(float angle, float speed, uint64_t nowMs);

private:
    static constexpr float Q      = 0.01f;
    static constexpr float Q_RATE = 0.005f;
    static constexpr float R      = 0.5f;

    float    state_[4];   // [ vx, vy, vx', vy' ]  Cartesian wind vector + rates
    float    P_[4][4];    // covariance matrix
    uint64_t lastTime_;
    bool     initialised_;

    // Core Kalman predict + update. Mutates state_ and P_ in-place.
    void step(float vx, float vy, uint64_t nowMs);

    static float normNautic(float a);   // wrap to [0, 2π)
};
