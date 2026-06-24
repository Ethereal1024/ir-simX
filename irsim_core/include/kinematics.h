#pragma once
#include "geometry.h"

// ═══════════════════════════════════════════════════════════════
//  Robot kinematics step
// ═══════════════════════════════════════════════════════════════

enum class KinematicsType : uint8_t {
    DIFF = 0,
    OMNI = 1,
    ACKER = 2,
    OMNI_ANGULAR = 3,
};

// All kinematics advance state (x, y, theta) by dt given control inputs.
// Controls are clamped to [vel_min, vel_max] by caller.

// diff: action[0]=v, action[1]=omega
void step_diff(float& x, float& y, float& theta,
               float v, float omega, float dt);

// omni: action[0]=vx, action[1]=vy, omega=0 (no independent yaw)
void step_omni(float& x, float& y, float& theta,
               float vx, float vy, float dt);

// omni_angular: action[0]=vx, action[1]=vy, action[2]=omega
void step_omni_angular(float& x, float& y, float& theta,
                       float vx, float vy, float omega, float dt);

void step_acker(float& x, float& y, float& theta,
                float v, float steer, float dt);

// Dispatch by type
void step_kinematics(KinematicsType type,
                     float& x, float& y, float& theta,
                     const float* action, float dt);
