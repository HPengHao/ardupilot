#include "AP_GPS.h"
//#include <random>
#include <GCS_MAVLink/GCS.h>

#define STL_ATK_TYPE 3

int getRandomInt(int a, int b){
    return (rand() % (b-a + 1)) + a;
}

void AP_GPS::get_fake_location(const Location& true_loc) const{

#if  STL_ATK_TYPE == 1
    //Attack type 1
    (*fake_loc_ptr) = true_loc;
    int range_down = 200;
    int range_up = 300;
    if(rand() % 2 == 0){
        fake_loc_ptr->lat += getRandomInt(range_down, range_up);
        fake_loc_ptr->lng += getRandomInt(range_down, range_up);
    }else{
        fake_loc_ptr->lat -= getRandomInt(range_down, range_up);
        fake_loc_ptr->lng -= getRandomInt(range_down, range_up);
    }

    // if(rand() % 5 == 0){
    //     gcs().send_text(MAV_SEVERITY_INFO, "random number: %d", getRandomInt(range_down, range_down));
    //     AP_HAL::get_HAL().console->printf("console random number: %d\n", getRandomInt(range_down, range_up));
    // }
#elif STL_ATK_TYPE == 2
    //Attack Type 2
    static Location fixed_loc = true_loc;
    static bool is_fix_loc = false;
    if(is_fix_loc){
        fixed_loc.lat -= 100;
        is_fix_loc = true;
    }
    (*fake_loc_ptr) = fixed_loc;

#elif STL_ATK_TYPE == 3
    static bool first = true;
    static Location last_fake_loc;
    if(first){
        (*fake_loc_ptr) = true_loc;
        fake_loc_ptr->lng += 100;
        last_fake_loc = (*fake_loc_ptr);
        first = false;
    }else{
        //only update latitude
        (*fake_loc_ptr) = last_fake_loc;
        fake_loc_ptr->lat = true_loc.lat;
        fake_loc_ptr->lng += getRandomInt(-10, 10);
    }

#endif
    //gcs().send_text(MAV_SEVERITY_INFO, "True data Lat: %d, Lng: %d", true_loc.lat, true_loc.lng);
    //gcs().send_text(MAV_SEVERITY_INFO, "Fake data Lat: %d, Lng: %d", fake_loc_ptr->lat, fake_loc_ptr->lng);
}