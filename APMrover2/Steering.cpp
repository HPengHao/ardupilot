#include "Rover.h"

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        // get ground speed
        float speed;
        if (!g2.attitude_control.get_forward_speed(speed)) {
            speed = 0.0f;
        }

        g2.motors.output(arming.is_armed(), speed, G_Dt);

        //==========compressed log ==================
        static bool delay_satrt = true;
        if(!delay_satrt || scheduler.ticks() > 150){
            delay_satrt = false;
            const uint64_t start_evaluation = AP_HAL::micros64();
            //Bob: log ekf data, -1 means the primary core;
            AP::ahrs_navekf().Log_Write_BKF1_rover(-1, AP_HAL::micros64());
            const uint64_t end_evaluation = AP_HAL::micros64();
            if(scheduler.ticks() % 50 == 0){
                AP::logger().Write_BOBL(19, (int)(end_evaluation-start_evaluation));
            }
        }
        //============================================

        //============peridically inter-sample attack=============
        int attack_period = 10 * 1000000; //unit: us
        int attack_duration = 1 * 1000000 / 2; //unit: us

        uint64_t now =AP_HAL::micros64();
        int mod_now = now % (attack_period);

        if(mod_now <= attack_duration){
            scheduler.inter_sample_atk = true;
        }else{
            scheduler.inter_sample_atk = false;
        }
        //========================================================
    }
}
