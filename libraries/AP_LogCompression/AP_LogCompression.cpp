#include "AP_LogCompression.h"

void AP_LOGC::quadrotor_m(float, const float x[12], const float u[4], float a, float b,
                          float c, float d, float m, float I_x, float I_y, float I_z,
                          float K_T, float K_Q, float dx[12], float y[12])
{
    int i;
    float b_x[16];

    // IDNLGREY model file (discrete-time nonlinear model) :
    // xn = x(t+Ts) : state update values in discrete-time case (A column vector with Nx entries)
    // y : outputs values (A column vector with Ny entries)
    //  gravity acceleration constant (m/s^2)
    //  inputs
    //      x(13:16)=u;
    for (i = 0; i < 12; i++)
    {
        b_x[i] = x[i];
    }

    for (i = 0; i < 4; i++)
    {
        b_x[i + 12] = u[i];
    }

    // -------------------------------------------------
    dx[0] = b_x[6];
    dx[1] = b_x[7];
    dx[2] = b_x[8];
    dx[3] = (b_x[9] + sinf(b_x[3]) * tanf(b_x[4]) * b_x[10]) + cosf(b_x[3]) * tanf(b_x[4]) * b_x[11];
    dx[4] = cosf(b_x[3]) * b_x[10] - sinf(b_x[3]) * b_x[11];
    dx[5] = sinf(b_x[3]) / cosf(b_x[4]) * b_x[10] + cosf(b_x[3]) / cosf(b_x[4]) * b_x[11];
    dx[6] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * cosf(b_x[5]) + sinf(b_x[3]) * sinf(b_x[5]));
    dx[7] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * sinf(b_x[4]) * sinf(b_x[5]) - sinf(b_x[3]) * cosf(b_x[5]));
    dx[8] = K_T / m * (((b_x[12] + b_x[13]) + b_x[14]) + b_x[15]) * (cosf(b_x[3]) * cosf(b_x[4])) - 9.80665F;
    dx[9] = (I_y - I_z) / I_x * b_x[10] * b_x[11] + K_T / I_x * (((-a * b_x[12] + d * b_x[13]) + a * b_x[14]) - d * b_x[15]);
    dx[10] = (I_x - I_z) / I_y * b_x[9] * b_x[11] + K_T / I_y * (((-b * b_x[12] + c * b_x[13]) - b * b_x[14]) + c * b_x[15]);
    dx[11] = (I_x - I_y) / I_z * b_x[9] * b_x[10] + K_Q / I_z * (((-b_x[12] - b_x[13]) + b_x[14]) + b_x[15]);

    
    //     %% addtional effects here
    //  air resistance
    //  *0.6538
    // rotational air resitance
    //  *0.2778
    for (i = 0; i < 3; i++)
    {
        dx[6 + i] += -x[6 + i] * 9.80665F / 15.0F;
        dx[9 + i] += -x[9 + i] * 6.98131704F / 25.1327419F;
    }
    float frame_height = 0.1;
    if ((x[2] - frame_height < 0.001F) && (dx[8] <= 0.0F))
    {
        //  on ground
        dx[8] = 0.0F;
    }

    //
    for (i = 0; i < 12; i++)
    {
        y[i] = x[i];
    }

    //  update outputs
}

void AP_LOGC::transfromNED2ENU(float state[12]){
    float x[12] = {0};
    x[0] = state[1];     x[1] = state[0];     x[2] = -state[2];
    x[3] = state[3];     x[4] = -state[4];    x[5] = wrap_2PI(-state[5] + M_PI/2);
    x[6] = state[7];    x[7] = state[6];    x[8] = -state[8];
    x[9] = state[9];    x[10] = -state[10];  x[11] = -state[11];
    for (int i = 0; i < 12; i++)
    {
        state[i] = x[i];
    }
    
}

void AP_LOGC::updateState(float x[12], float dx[12], float dt){
    for (int i = 0; i < 12; i++)
    {
        x[i] += dx[i] * dt;
    }
    x[5] = wrap_2PI(x[5]);
    
}

void AP_LOGC::compressionLog(const Vector3f& velNED, const Vector2f& posNE, float posD, 
        const Vector3f& gyroUnbias, const Vector3f& euler, uint16_t time_us, const float* motor_actuators_data){
    static float x[12] = {0};
    static float dx[12] = {0};
    static float y[12] = {0};
    static float u[4] = {0};
    static float a = 0.128364;
    static float b = 0.128364;
    static float c = 0.128364;
    static float d = 0.128364;
    static float m = 1.5;
    static float I_x = 0.015;
    static float I_y = 0.015;
    static float I_z = 0.015;
    static float K_T = 7.21077; 
    static float K_Q = 0.10472; 
    static uint64_t last_time_us = 0;
    
    //prepare dt
    float dt = (float)((double)(time_us-last_time_us)*1e-6);
    last_time_us = time_us;

    //prepare input
    for (int i = 0; i < 4; i++)
    {
        u[i] = motor_actuators_data[i];
    }

    AP_LOGC::updateState(x, dx, dt); //1. update old states to predict current states.

    //2. test if we need synchronization. If so, synchronize. //TODO
    //for testing running time, we just sychronize all the time.
    x[0] = posNE.x;         x[1] = posNE.y;         x[2] = posD;
    x[3] = euler.x;         x[4] = euler.y;         x[5] = wrap_2PI(euler.z);
    x[6] = velNED.x;        x[7] = velNED.y;        x[8] = velNED.z;
    x[9] = gyroUnbias.x;    x[10] = gyroUnbias.y;   x[11] = gyroUnbias.z;
    AP_LOGC::transfromNED2ENU(x);

    //3. get current output and calculate dt for next time.
    AP_LOGC::quadrotor_m(0.0, x, u, a, b, c, d, m, I_x, I_y, I_z, K_T, K_Q, dx, y);

}