#include "AP_LogCompression.h"
#include <AP_Logger/AP_Logger.h>

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

void AP_LOGC::compressionLog(const struct log_Bob_EKF1 & sensor_pkt, const struct log_motors & motor_pkt){
    //parameters
    static float x[12] = {0};
    static float true_x[12] = {0};
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
    static int K_stone = 400;
    static int loopCount = -1;
    static int max_freq = 400;
    
    static float error_thre[12] = {0.139337476507485, 0.124396874563097, 0.0147272946765464,
                                0.00372856443706101, 0.00390555433968539, 0.00589400093019354, 
                                0.0696205367431640, 0.0815225658620695, 0.0264938590124031, 
                                0.00822311394035166, 0.00840920644259765, 0.0101633185357302};
    static int last_log_loop[12] = {0};

    uint64_t time_us = sensor_pkt.time_us;
    
    //prepare dt, unit: s
    float dt = (float)((double)(time_us-last_time_us)*1e-6);
    last_time_us = time_us;

    //prepare input
    u[0] = transformInput(motor_pkt.motor1);
    u[1] = transformInput(motor_pkt.motor2);
    u[2] = transformInput(motor_pkt.motor3);
    u[3] = transformInput(motor_pkt.motor4);

    //1. update old states to predict current states.
    AP_LOGC::updateState(x, dx, dt); 

    true_x[0] = sensor_pkt.posN;         true_x[1] = sensor_pkt.posE;         true_x[2] = sensor_pkt.posD;
    true_x[3] = sensor_pkt.roll;         true_x[4] = sensor_pkt.pitch;         true_x[5] = wrap_2PI(sensor_pkt.yaw);
    true_x[6] = sensor_pkt.velN;        true_x[7] = sensor_pkt.velE;        true_x[8] = sensor_pkt.velD;
    true_x[9] = sensor_pkt.gyrX;    true_x[10] = sensor_pkt.gyrY;   true_x[11] = sensor_pkt.gyrZ;
    AP_LOGC::transfromNED2ENU(true_x);

    //2. test if we need synchronization. If so, synchronize. 
    loopCount++; //loopCount change from [0, K_stone)
    if(loopCount % K_stone == 0){ //2.1 K-milestone sychronization
        loopCount = 0;
        for (int i = 0; i < 12; i++)
        {
            x[i] = true_x[i];
            last_log_loop[i] = loopCount;
        }
        struct log_Bob_EKF1 log_sycn = sensor_pkt;
        log_sycn.msgid = LOG_CLOG_SYN_MSG;
        AP::logger().WriteCriticalBlock(&log_sycn, sizeof(log_sycn));
    }

    //3. get current output and calculate dx for next time.
    AP_LOGC::quadrotor_m(0.0, x, u, a, b, c, d, m, I_x, I_y, I_z, K_T, K_Q, dx, y);
    
    //compression log
    for (int i = 0; i < 12; i++)
    {
        float error = abs(true_x[i] - y[i]);
        if(is_log(error, error_thre[i], last_log_loop[i], loopCount, max_freq)){
            //need to log
            AP::logger().WriteCritical("CLOG", "TimeUS,stateNo,value", "Qbf",
                                    time_us,
                                    (int8_t)(i+1),
                                    (float)true_x[i]
            );
            last_log_loop[i] = loopCount;
        }
    }

}

bool AP_LOGC::is_log(float error, float error_max, int last_log_loop, int current_loop, int max_freq){
    float scale_factor = 1;
    if(error < 1e-20 || last_log_loop == current_loop){
        return false;
    }
    if(error >= error_max){
        return true;
    }else{
        float desired_freq = scale_factor * (error / error_max) * max_freq;
        float current_freq = max_freq * (1/(float)(current_loop - last_log_loop));
        if(desired_freq > current_freq){
            return true;
        }else
        {
            return false;
        }
        
    }
}

float AP_LOGC::transformInput(float actuator){
    return (((actuator*1000)+1000)-1100)/900;
}