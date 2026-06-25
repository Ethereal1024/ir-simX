#include "kinematics.h"
#include <cmath>

void step_diff(float& x, float& y, float& theta,
               float v, float omega, float dt)
{
    if (std::abs(omega) < 1e-8f) {
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
    } else {
        float r = v / omega;
        float theta_new = theta + omega * dt;
        x += r * (std::sin(theta_new) - std::sin(theta));
        y += r * (std::cos(theta) - std::cos(theta_new));
        theta = theta_new;
    }
}

void step_omni(float& x, float& y, float& theta,
               float vx, float vy, float dt)
{
    // No omega control; theta unchanged
    x += (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
    y += (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
}

void step_omni_angular(float& x, float& y, float& theta,
                       float vx, float vy, float omega, float dt)
{
    x += (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
    y += (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
    theta += omega * dt;
}

void step_acker(float& x, float& y, float& theta, float& steer_angle,
                float v, float desired_steer, float dt, float wheelbase)
{
    float omega = 0.0f;
    if (std::abs(steer_angle) > 1e-8f) {
        // Use current steer angle for turn rate, matching Python semantics
        omega = v * std::tan(steer_angle) / wheelbase;
    }
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta += omega * dt;
    // Update steer angle to desired value for next step
    steer_angle = desired_steer;
}

void step_kinematics(KinematicsType type,
                     float& x, float& y, float& theta,
                     float* steer_angle, float wheelbase,
                     const float* action, float dt)
{
    switch (type) {
    case KinematicsType::DIFF:
        step_diff(x, y, theta, action[0], action[1], dt);
        break;
    case KinematicsType::OMNI:
        step_omni(x, y, theta, action[0], action[1], dt);
        break;
    case KinematicsType::OMNI_ANGULAR:
        step_omni_angular(x, y, theta, action[0], action[1], action[2], dt);
        break;
    case KinematicsType::ACKER:
        step_acker(x, y, theta, *steer_angle, action[0], action[1], dt, wheelbase);
        break;
    }
}
