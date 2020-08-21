#include "AP_GPS.h"
#include <random>


const Location& AP_GPS::get_fake_location(const Location& true_loc) const{
    Location fake_loc(true_loc);
    static std::random_device rd;  //Will be used to obtain a seed for the random number engine
    static std::mt19937 rgen(rd()); //Standard mersenne_twister_engine seeded with rd()
    static std::uniform_int_distribution<int32_t> uni_dis(5,10);
    static std::uniform_real_distribution<> uni_sign(0.0,1.0);
    if(uni_sign(rgen) > 0.5){
        fake_loc.lat += uni_dis(rgen);
        fake_loc.lng += uni_dis(rgen);
    }else{
        fake_loc.lat -= uni_dis(rgen);
        fake_loc.lng -= uni_dis(rgen);
    }
    return fake_loc;
}