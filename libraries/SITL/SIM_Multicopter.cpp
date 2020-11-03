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
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>
#include "fileOperation.h"

#include <stdio.h>

using namespace SITL;

MultiCopter::MultiCopter(const char *frame_str) :
    Aircraft(frame_str),
    frame(nullptr)
{
    mass = 1.5f;

    frame = Frame::find_frame(frame_str);
    if (frame == nullptr) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    // initial mass is passed through to Frame for it to calculate a
    // hover thrust requirement.
    if (strstr(frame_str, "-fast")) {
        frame->init(gross_mass(), 0.5, 85, 4*radians(360));
    } else {
        frame->init(gross_mass(), 0.51, 15, 4*radians(360));
    }
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    std::string fileNo = "00000247";
    std::string data_folder = "/home/bob/ardupilot/libraries/SITL/sim_rerun/MultiCopter/";
    std::string disturb_filePath = data_folder + fileNo + "_disturb.csv";
    std::string syn_filePath = data_folder + fileNo + "_syn.csv";
    readCSV(disturb_filePath, disturb_data);
    readCSV(syn_filePath, sync_data);
    if(disturb_data.size() > 0)
        printf("disturb data lines: (%d, %d), %f\n", (int)disturb_data.size(), (int) disturb_data[0].size(), disturb_data[0][0]);
    if(sync_data.size() > 0)
        printf("sycn data lines: (%d, %d), %f\n", (int)sync_data.size(), (int) sync_data[0].size(), sync_data[0][0]);
}

void MultiCopter::add_disturb_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel){
    static bool armed = false;
    static uint64_t arm_time;
    if(!armed && input.servos[0] > 1010){
        armed = true;
        arm_time = time_now_us;
    }
    if(!armed){
        return;
    }
    static int idx = 0;
    if(idx >= (int)disturb_data.size()){
        return;
    }
    if(time_now_us-arm_time > (uint64_t) disturb_data[idx][0]){
        while((uint64_t) disturb_data[idx][0] < time_now_us-arm_time){
            idx++;
            if(idx >= (int)disturb_data.size()){
                return;
            }
        }
        idx--;// find the closest disturbance time less than time_now_us
        rot_accel.x += (float)disturb_data[idx][4];
        rot_accel.y += (float)disturb_data[idx][5];
        rot_accel.z += (float)disturb_data[idx][6];
        // earth frame linear accel (disturbance)
        Vector3f lin_accel_ef = Vector3f((float)disturb_data[idx][1], 
                                        (float)disturb_data[idx][2], 
                                        (float)disturb_data[idx][3]);
        body_accel += get_dcm().transposed() * lin_accel_ef;
    }
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
    add_disturb_forces(input, rot_accel, body_accel);
    add_shove_forces(rot_accel, body_accel);
    add_twist_forces(rot_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    // estimate voltage and current
    frame->current_and_voltage(input, battery_voltage, battery_current);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
