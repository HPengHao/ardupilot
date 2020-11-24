/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  rover simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "fileOperation.h"

namespace SITL {

/*
  a rover simulator
 */
class SimRover : public Aircraft {
public:
    SimRover(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new SimRover(frame_str);
    }

private:
    float max_speed;
    float max_accel;
    float max_wheel_turn;
    float turning_circle;
    float skid_turn_rate;
    bool skid_steering;

    float turn_circle(float steering);
    float calc_yaw_rate(float steering, float speed);
    float calc_lat_accel(float steering_angle, float speed);
    std::vector<std::vector<double>> disturb_data_lin;
    std::vector<std::vector<double>> disturb_data_rot;
    std::vector<std::vector<double>> disturb_data_arr[2];
    std::vector<std::vector<double>> sync_data;
    std::vector<std::vector<double>> config_data;
    bool is_origin_model = true;
    bool is_add_disturb = false;
    bool is_last_origin = true;
    uint64_t arm_time;
    bool armed = false;

    //parameters
    float x[6] = {0}; //velocity in body Frame, position in NED frame
    float dx[6] = {0};
    float y_out[6] = {0}; //velocity in earth Frame
    float u[2] = {0};
    float m = 1.7;
    float a = 0.5;
    float b = 0.7;
    float Cx = 17;
    float Cy = 17;
    float CA = 0.1706;

    void new_model_step(const struct sitl_input &input);
    void state_sycn_origin2new();
    void state_sycn_new2origin();
};

} // namespace SITL
