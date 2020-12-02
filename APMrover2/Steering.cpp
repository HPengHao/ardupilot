#include "Rover.h"

/*
    work out if we are going to use pivot steering
*/
bool Rover::use_pivot_steering(float yaw_error_cd)
{
    // check cases where we clearly cannot use pivot steering
    if (!g2.motors.have_skid_steering() || g.pivot_turn_angle <= 0) {
        pivot_steering_active = false;
        return false;
    }

    // calc bearing error
    const float yaw_error = fabsf(yaw_error_cd) * 0.01f;

    // if error is larger than pivot_turn_angle start pivot steering
    if (yaw_error > g.pivot_turn_angle) {
        pivot_steering_active = true;
        return true;
    }

    // if within 10 degrees of the target heading, exit pivot steering
    if (yaw_error < 10.0f) {
        pivot_steering_active = false;
        return false;
    }

    // by default stay in
    return pivot_steering_active;
}

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        g2.motors.output(arming.is_armed() && hal.util->get_soft_armed(), G_Dt);
    }

    //==========compressed log ==================
        static bool delay_satrt = true;
        if(!delay_satrt || scheduler._tick_counter > 150){
            delay_satrt = false;
            const uint64_t start_evaluation = AP_HAL::micros64();
            //Bob: log ekf data, -1 means the primary core;
            ahrs.Log_Write_BKF1_rover(-1, AP_HAL::micros64());
            const uint64_t end_evaluation = AP_HAL::micros64();
            if(scheduler._tick_counter % 50 == 0){
                DataFlash.Write_BOBL(19, (int)(end_evaluation-start_evaluation));
            }
        }
    //============================================
}
