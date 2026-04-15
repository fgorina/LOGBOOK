#include "WindKalman.h"
#include <cstring>

WindKalmanPolar::WindKalmanPolar()
    : lastTime_(0), initialised_(false)
{
    memset(state_, 0, sizeof(state_));
    memset(P_,     0, sizeof(P_));
    P_[0][0] = P_[1][1] = 1.0f;
    P_[2][2] = P_[3][3] = 1.0f;
}

float WindKalmanPolar::normNautic(float a)
{
    a = fmodf(a, 2.0f * (float)M_PI);
    return a < 0.0f ? a + 2.0f * (float)M_PI : a;
}

// ─── Core Cartesian Kalman step ──────────────────────────────────────────────
void WindKalmanPolar::step(float vx, float vy, uint64_t nowMs)
{
    if (!initialised_) {
        state_[0] = vx;   state_[1] = vy;
        state_[2] = 0.0f; state_[3] = 0.0f;
        lastTime_    = nowMs;
        initialised_ = true;
        return;
    }

    float dt = (nowMs - lastTime_) / 1000.0f;
    lastTime_ = nowMs;

    if (dt <= 0.0f || dt > 30.0f) {
        // Time discontinuity — reinitialise to current measurement
        state_[0] = vx;   state_[1] = vy;
        state_[2] = 0.0f; state_[3] = 0.0f;
        return;
    }

    // ── Predict ──────────────────────────────────────────────────────────────
    float px  = state_[0] + state_[2] * dt;
    float py  = state_[1] + state_[3] * dt;
    float pvx = state_[2];
    float pvy = state_[3];

    float dt2 = dt * dt;
    float PP[4][4];

    PP[0][0] = P_[0][0] + dt*(P_[2][0]+P_[0][2]) + dt2*P_[2][2] + Q*dt2;
    PP[0][1] = P_[0][1] + dt*(P_[2][1]+P_[0][3]) + dt2*P_[2][3];
    PP[0][2] = P_[0][2] + dt*P_[2][2] + Q*dt;
    PP[0][3] = P_[0][3] + dt*P_[2][3];

    PP[1][0] = P_[1][0] + dt*(P_[3][0]+P_[1][2]) + dt2*P_[3][2];
    PP[1][1] = P_[1][1] + dt*(P_[3][1]+P_[1][3]) + dt2*P_[3][3] + Q*dt2;
    PP[1][2] = P_[1][2] + dt*P_[3][2];
    PP[1][3] = P_[1][3] + dt*P_[3][3] + Q*dt;

    PP[2][0] = P_[2][0] + dt*P_[2][2] + Q_RATE*dt;
    PP[2][1] = P_[2][1] + dt*P_[2][3];
    PP[2][2] = P_[2][2] + Q_RATE;
    PP[2][3] = P_[2][3];

    PP[3][0] = P_[3][0] + dt*P_[3][2];
    PP[3][1] = P_[3][1] + dt*P_[3][3] + Q_RATE*dt;
    PP[3][2] = P_[3][2];
    PP[3][3] = P_[3][3] + Q_RATE;

    // ── Update ───────────────────────────────────────────────────────────────
    float S00 = PP[0][0] + R,  S01 = PP[0][1];
    float S10 = PP[1][0],      S11 = PP[1][1] + R;

    float detS = S00*S11 - S01*S10;
    if (fabsf(detS) < 1e-10f) detS = 1e-10f;

    float Si00 =  S11/detS,  Si01 = -S01/detS;
    float Si10 = -S10/detS,  Si11 =  S00/detS;

    float K[4][2];
    for (int i = 0; i < 4; i++) {
        K[i][0] = PP[i][0]*Si00 + PP[i][1]*Si10;
        K[i][1] = PP[i][0]*Si01 + PP[i][1]*Si11;
    }

    float innov_x = vx - px;
    float innov_y = vy - py;

    state_[0] = px  + K[0][0]*innov_x + K[0][1]*innov_y;
    state_[1] = py  + K[1][0]*innov_x + K[1][1]*innov_y;
    state_[2] = pvx + K[2][0]*innov_x + K[2][1]*innov_y;
    state_[3] = pvy + K[3][0]*innov_x + K[3][1]*innov_y;

    for (int i = 0; i < 4; i++) {
        P_[i][0] = PP[i][0] - K[i][0]*PP[0][0] - K[i][1]*PP[1][0];
        P_[i][1] = PP[i][1] - K[i][0]*PP[0][1] - K[i][1]*PP[1][1];
        P_[i][2] = PP[i][2] - K[i][0]*PP[0][2] - K[i][1]*PP[1][2];
        P_[i][3] = PP[i][3] - K[i][0]*PP[0][3] - K[i][1]*PP[1][3];
    }
}

// ─── Public update: polar → Cartesian → Kalman → polar ───────────────────────
WindKalmanResult WindKalmanPolar::update(float angle, float speed, uint64_t nowMs)
{
    // Nautical polar → Cartesian  (N=0, clockwise → East=x, North=y)
    float vx = speed * sinf(angle);
    float vy = speed * cosf(angle);

    step(vx, vy, nowMs);

    float svx = state_[0], svy = state_[1];
    float svx_r = state_[2], svy_r = state_[3];

    float eps   = 1e-6f;
    float mag   = sqrtf(svx*svx + svy*svy);
    float ang   = atan2f(svx, svy);          // nautical atan2

    float mag_r = (svx*svx_r + svy*svy_r) / (mag + eps);
    float ang_r = (svy*svx_r - svx*svy_r) / (mag*mag + eps);

    return WindKalmanResult{
        normNautic(ang),
        mag,
        ang_r,
        mag_r
    };
}
