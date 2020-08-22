#include "AP_GPS.h"
#include <random>
#include <GCS_MAVLink/GCS.h>

#define STL_ATK_TYPE 2
void AP_GPS::get_fake_location(const Location& true_loc) const{
#if  STL_ATK_TYPE == 1
    //Attack type 1
    (*fake_loc_ptr) = true_loc;
    static std::random_device rd;  //Will be used to obtain a seed for the random number engine
    static std::mt19937 rgen(rd()); //Standard mersenne_twister_engine seeded with rd()
    static std::uniform_int_distribution<int32_t> uni_dis(200,300);
    static std::uniform_real_distribution<> uni_sign(0.0,1.0);
    if(uni_sign(rgen) > 0.5){
        fake_loc_ptr->lat += uni_dis(rgen);
        fake_loc_ptr->lng += uni_dis(rgen);
    }else{
        fake_loc_ptr->lat -= uni_dis(rgen);
        fake_loc_ptr->lng -= uni_dis(rgen);
    }
#elif STL_ATK_TYPE == 2
    //Attack Type 2
    static Location fixed_loc = true_loc;
    static bool is_fix_loc = false;
    if(is_fix_loc){
        fixed_loc.lat -= 100;
        is_fix_loc = true;
    }
    (*fake_loc_ptr) = fixed_loc;

#endif
    //gcs().send_text(MAV_SEVERITY_INFO, "True data Lat: %d, Lng: %d", true_loc.lat, true_loc.lng);
    //gcs().send_text(MAV_SEVERITY_INFO, "Fake data Lat: %d, Lng: %d", fake_loc_ptr->lat, fake_loc_ptr->lng);
    //return fake_loc;
}