#pragma once

#include <cstdlib>
#include <AP_Math/AP_Math.h>


namespace AP_LOGC{
    void quadrotor_m(float, const float x[12], const float u[4], float a, float
                 b, float c, float d, float m, float I_x, float I_y, float
                 I_z, float K_T, float K_Q, float dx[12], float y[12]);
    
    void transfromNED2ENU(float x[12]);
    void updateState(float x[12], float dx[12], float dt);
    void compressionLog(const Vector3f& velNED, const Vector2f& posNE, float posD, 
        const Vector3f& gyroUnbias, const Vector3f& euler, uint16_t time_us, const float* motor_actuators_data);
}