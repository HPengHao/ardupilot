#pragma once
#include <AP_Math/AP_Math.h>
#include <cmath>
#include <stdio.h>
#include <list>

#include "model/fdeep_angvelx.h"
#include "model/fdeep_angvely.h"
#include "model/fdeep_angvelz.h"
#include "model/fdeep_velx.h"
#include "model/fdeep_vely.h"
#include "model/fdeep_velz.h"
#include "model/fdeep_posx.h"
#include "model/fdeep_posy.h"
#include "model/fdeep_posz.h"
#include "model/fdeep_roll.h"
#include "model/fdeep_pitch.h"
#include "model/fdeep_yaw.h"

#include<iostream>
#include<fstream>

//only posy predict and detect and recover now.
class RECOVER
{
private:
    Eigen::MatrixXf relu(const Eigen::MatrixXf& matrix);
    Eigen::MatrixXf nom(const Eigen::MatrixXf& matrix, float MaxNum, float MinNum);
    Eigen::MatrixXf inom(const Eigen::MatrixXf& matrix, float MaxNum, float MinNum);
    float nom(float a, float MaxNum, float MinNum);
    float inom(float a, float MaxNum, float MinNum);

    FDEEPANGVELX fdeepangvelx;
    FDEEPANGVELY fdeepangvely;
    FDEEPANGVELZ fdeepangvelz;
    FDEEPVELX fdeepvelx;
    FDEEPVELY fdeepvely;
    FDEEPVELZ fdeepvelz;
    FDEEPPOSX fdeepposx;
    FDEEPPOSY fdeepposy;
    FDEEPPOSZ fdeepposz;
    FDEEPROLL fdeeproll;
    FDEEPPITCH fdeeppitch;
    FDEEPYAW fdeepyaw;

public:
    RECOVER(){
    }

    //只会有一个PREDICT实例，可以通过getinstance()函数调用
    static RECOVER* get_instance(){
        static RECOVER instance;
        return &instance;
    }

    //用于速度转换为加速度的循环次数
    const int numberOfIterations = 400;
    bool bigger10 = false;
    //用于获取坐标系转换的旋转矩阵
    Quaternion q;
    std::list<int> previousValues; // 使用list保存前几次循环的数值

    //目标值
    Vector3d targetpos;
    Vector3f targetvel;
    Vector3f targetangvel;
    Vector3f targetattitude;
    
    //以下虽然有速度对应值，但实际无速度恢复部分代码，速度用于计算加速度
    //预测值
    float MLposx = 0.0f;
    float MLposy = 0.0f;
    float MLposz = 0.0f;
    float MLvelx = 0.0f;
    float MLvely = 0.0f;
    float MLvelz = 0.0f;
    float MLdeltavelx = 0.0f;
    float MLdeltavely = 0.0f;
    float MLdeltavelz = 0.0f;
    float MLangvelx = 0.0f;
    float MLangvely = 0.0f;
    float MLangvelz = 0.0f;
    float MLroll = 0.0f;
    float MLpitch = 0.0f;
    float MLyaw = 0.0f;
    //恢复值
    float posx = 0.0f;
    float posy = 0.0f;
    float posz = 0.0f;
    float velx = 0.0f;
    float vely = 0.0f;
    float velz = 0.0f;
    float deltavelx = 0.0f;
    float deltavely = 0.0f;
    float deltavelz = 0.0f;
    float angvelx = 0.0f;
    float angvely = 0.0f;
    float angvelz = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    //恢复标志
    bool recover_posx=false;
    bool recover_posy=false;
    bool recover_posz=false;
    bool recover_velx=false;
    bool recover_vely=false;
    bool recover_velz=false;
    bool recover_deltavelx=false;
    bool recover_deltavely=false;
    bool recover_deltavelz=false;
    bool recover_angvelx=false;
    bool recover_angvely=false;
    bool recover_angvelz=false;
    bool recover_roll=false;
    bool recover_pitch=false;
    bool recover_yaw=false;

    // 是否armed
    bool is_armed=false;

    //通过当前值和目标值进行预测
    float predict(float target, float real, std::string type);
    //通过预测值和实际值进行判断是否被攻击，进行恢复
    void detect_and_recover(float time, float target, float real, float &predict_data, float &recover_data, bool &recover_flag ,std::string type, 
    float& ml_err_sum, float& ml_max_mse, int& w_index, int window, float threshold_err, float threshold_rec, int& err_add, int err_times, int& rec_add, int rec_times);


    void posxrecoevery(float real);
    void posyrecoevery(float real);
    void poszrecoevery(float real);
    void velxrecoevery(float real);
    void velyrecoevery(float real);
    void velzrecoevery(float real);
    void deltavelxrecoevery(float time, float real);
    void deltavelyrecoevery(float time, float real);
    void deltavelzrecoevery(float time, float real);
    void angvelxrecoevery(float real);
    void angvelyrecoevery(float real);
    void angvelzrecoevery(float real);
    void rollrecoevery(float real);
    void pitchrecoevery(float real);
    void yawrecoevery(float real);

};
