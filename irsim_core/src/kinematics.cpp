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

void step_acker(float& x, float& y, float& theta,
                float v, float steer, float dt)
{
    float L = 0.5f;
    if (std::abs(steer) < 1e-8f) {
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
    } else {
        float R = L / std::tan(steer);
        float omega = v / R;
        step_diff(x, y, theta, v, omega, dt);
    }
}

void step_kinematics(KinematicsType type,
                     float& x, float& y, float& theta,
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
        step_acker(x, y, theta, action[0], action[1], dt);
        break;
    }
}
