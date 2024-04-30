#include "recover.h"

Eigen::MatrixXf RECOVER::relu(const Eigen::MatrixXf& matrix) {
    Eigen::MatrixXf result(matrix.rows(), matrix.cols());

    for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
            result(i, j) = matrix(i, j) > 0 ? matrix(i, j) : 0;
        }
    }

    return result;
}

Eigen::MatrixXf RECOVER::nom(const Eigen::MatrixXf& matrix, float MaxNum, float MinNum) {
    Eigen::MatrixXf result(matrix.rows(), matrix.cols());

    for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
            result(i, j) = (matrix(i, j) - MinNum) / (MaxNum - MinNum);
            if(matrix(i, j) > MaxNum)
                result(i, j) = 1;
            else if(matrix(i, j) < MinNum)
                result(i, j) = 0;
        }
    }

    return result;
}

float RECOVER::nom(float a, float MaxNum, float MinNum){
    float result;
    result = (a - MinNum) / (MaxNum - MinNum);
    if(a > MaxNum)
        result = 1;
    else if(a < MinNum)
        result = 0;
    return result;
}

Eigen::MatrixXf RECOVER::inom(const Eigen::MatrixXf& matrix, float MaxNum, float MinNum) {
    Eigen::MatrixXf result(matrix.rows(), matrix.cols());

    for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
            result(i, j) = matrix(i, j) * (MaxNum - MinNum) + MinNum;
            if(matrix(i, j) > 1)
                result(i, j) = MaxNum;
            else if(matrix(i, j) < 0)
                result(i, j) = MinNum;
        }
    }

    return result;
}

float RECOVER::inom(float a, float MaxNum, float MinNum){
    float result;
    result = a * (MaxNum - MinNum) + MinNum;
    if(a > 1)
        result = MaxNum;
    else if(a < 0)
        result = MinNum;
    return result;
}

static float cal_error(float p, float r)
{
    return abs(p-r);
}

float RECOVER::predict(float target, float real, std::string type) {

    //输入的2*1的矩阵(目标值，当前值)
    Eigen::Matrix<float, 1, 2> input;
    input << target, real;
    Eigen::MatrixXf output;
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;
    float max;
    float min;
    if (type == "posx"){
        weightss = fdeepposx.weightss;
        biasess = fdeepposx.biasess;
        max = 2200;
        min = -2300;
    }
    else if (type == "posy"){
        weightss = fdeepposy.weightss;
        biasess = fdeepposy.biasess;
        max = 2600;
        min = -2500;
    }
    else if (type == "posz"){
        weightss = fdeepposz.weightss;
        biasess = fdeepposz.biasess;
        max = 2000;
        min = 0;
    }
    else if (type == "velx"){
        weightss = fdeepvelx.weightss;
        biasess = fdeepvelx.biasess;
        max = 2500;
        min = -2500;
    }
    else if (type == "vely"){
        weightss = fdeepvely.weightss;
        biasess = fdeepvely.biasess;
        max = 2500;
        min = -2500;
    }
    else if (type == "velz"){
        weightss = fdeepvelz.weightss;
        biasess = fdeepvelz.biasess;
        max = 2500;
        min = -2500;
    }
    else if (type == "angvelx"){
        weightss = fdeepangvelx.weightss;
        biasess = fdeepangvelx.biasess;
        max = 2500;
        min = -2500;
    }
    else if (type == "angvely"){
        weightss = fdeepangvely.weightss;
        biasess = fdeepangvely.biasess;
        max = 2500;
        min = -2500;
    }
    else if (type == "angvelz"){
        weightss = fdeepangvelz.weightss;
        biasess = fdeepangvelz.biasess;
        max = 2500;
        min = -2500;
    }
    else if (type == "roll"){
        weightss = fdeeproll.weightss;
        biasess = fdeeproll.biasess;
        max = 1600;
        min = -2700;
    }
    else if (type == "pitch"){
        weightss = fdeeppitch.weightss;
        biasess = fdeeppitch.biasess;
        max = 1600;
        min = -3100;
    }
    else if (type == "yaw"){
        weightss = fdeepyaw.weightss;
        biasess = fdeepyaw.biasess;
        max = 35999;
        min = 0;
    }
    else{
        printf("type error\n");
        return 0;
    }
    
    output = nom(input,max,min);

    unsigned long int i;
    for (i = 0; i < weightss.size(); i++)
    {
        
        output = (output * weightss[i]+ biasess[i]);
        output = relu(output);
    }
        output = inom(output,max,min);

    return output(0,0);
}

static const int  ml_roll_window = 1700;
static const int  ml_pitch_window = 2000;
static const int  ml_yaw_window = 4500;
static const int  ml_posx_window = 4400;
static const int  ml_posy_window = 5000;
static const int  ml_posz_window = 5000;
static const int  ml_velx_window = 4500;
static const int  ml_vely_window = 4300;
static const int  ml_velz_window = 5400;
static const int  ml_deltavelx_window = 4500;
static const int  ml_deltavely_window = 4300;
static const int  ml_deltavelz_window = 5400;
static const int  ml_angvelx_window = 2900;
static const int  ml_angvely_window = 4600;
static const int  ml_angvelz_window = 4200;

static const float  ml_roll_threshold = 50967.593750;
static const float  ml_pitch_threshold = 56562.242188;
static const float  ml_yaw_threshold = 80000;
static const float  ml_posx_threshold = 12;
static const float  ml_posy_threshold = 0.14;
static const float  ml_posz_threshold = 0.03;
static const float  ml_velx_threshold = 0.2;
static const float  ml_vely_threshold = 0.2;
static const float  ml_velz_threshold = 2;
static const float  ml_deltavelx_threshold = 10;
static const float  ml_deltavely_threshold = 10;
static const float  ml_deltavelz_threshold = 2;
static const float  ml_angvelx_threshold = 0.009 ;
static const float  ml_angvely_threshold = 0.01 ;
static const float  ml_angvelz_threshold = 0.6 ;

static const float  ml_roll_threshold_rec = 50967.593750 ;
static const float  ml_pitch_threshold_rec = 56562.242188 ;
static const float  ml_yaw_threshold_rec = 80000 ;
static const float  ml_posx_threshold_rec = 12;
static const float  ml_posy_threshold_rec = 0.14;
static const float  ml_posz_threshold_rec = 0.03;
static const float  ml_velx_threshold_rec = 0.2;
static const float  ml_vely_threshold_rec = 0.2;
static const float  ml_velz_threshold_rec = 2;
static const float  ml_deltavelx_threshold_rec = 10;
static const float  ml_deltavely_threshold_rec = 10;
static const float  ml_deltavelz_threshold_rec = 2;
static const float  ml_angvelx_threshold_rec = 0.009 ;
static const float  ml_angvely_threshold_rec = 0.01 ;
static const float  ml_angvelz_threshold_rec = 0.6 ;

static float    ml_roll_err_sum = 0;
static float    ml_pitch_err_sum = 0;
static float    ml_yaw_err_sum = 0;
static float    ml_posx_err_sum = 0;
static float    ml_posy_err_sum = 0;
static float    ml_posz_err_sum = 0;
static float    ml_velx_err_sum = 0;
static float    ml_vely_err_sum = 0;
static float    ml_velz_err_sum = 0;
static float    ml_deltavelx_err_sum = 0;
static float    ml_deltavely_err_sum = 0;
static float    ml_deltavelz_err_sum = 0;
static float    ml_angvelx_err_sum = 0;
static float    ml_angvely_err_sum = 0;
static float    ml_angvelz_err_sum = 0;

static int    ml_roll_w_index = 0;// current index in window
static int    ml_pitch_w_index = 0;
static int    ml_yaw_w_index = 0;
static int    ml_posx_w_index = 0;
static int    ml_posy_w_index = 0;
static int    ml_posz_w_index = 0;
static int    ml_velx_w_index = 0;
static int    ml_vely_w_index = 0;
static int    ml_velz_w_index = 0;
static int    ml_deltavelx_w_index = 0;
static int    ml_deltavely_w_index = 0;
static int    ml_deltavelz_w_index = 0;
static int    ml_angvelx_w_index = 0;
static int    ml_angvely_w_index = 0;
static int    ml_angvelz_w_index = 0;

static float    ml_roll_max_mse = 0;// max error (for logs)
static float    ml_pitch_max_mse = 0;
static float    ml_yaw_max_mse = 0;
static float    ml_posx_max_mse = 0;
static float    ml_posy_max_mse = 0;
static float    ml_posz_max_mse = 0;
static float    ml_velx_max_mse = 0;
static float    ml_vely_max_mse = 0;
static float    ml_velz_max_mse = 0;
static float    ml_deltavelx_max_mse = 0;
static float    ml_deltavely_max_mse = 0;
static float    ml_deltavelz_max_mse = 0;
static float    ml_angvelx_max_mse = 0;
static float    ml_angvely_max_mse = 0;
static float    ml_angvelz_max_mse = 0;

static int posx_error_add=0;
static int posy_error_add=0;
static int posz_error_add=0;
static int roll_error_add=0;
static int pitch_error_add=0;
static int yaw_error_add=0;
static int velx_error_add=0;
static int vely_error_add=0;
static int velz_error_add=0;
static int deltavelx_error_add=0;
static int deltavely_error_add=0;
static int deltavelz_error_add=0;
static int angvelx_error_add=0;
static int angvely_error_add=0;
static int angvelz_error_add=0;

static int posx_recover_add=0;
static int posy_recover_add=0;
static int posz_recover_add=0;
static int roll_recover_add=0;
static int pitch_recover_add=0;
static int yaw_recover_add=0;
static int velx_recover_add=0;
static int vely_recover_add=0;
static int velz_recover_add=0;
static int deltavelx_recover_add=0;
static int deltavely_recover_add=0;
static int deltavelz_recover_add=0;
static int angvelx_recover_add=0;
static int angvely_recover_add=0;
static int angvelz_recover_add=0;

static int posx_error_times=10;
static int posy_error_times=10;
static int posz_error_times=10;
static int roll_error_times=50;
static int pitch_error_times=50;
static int yaw_error_times=50;
static int velx_error_times=10;
static int vely_error_times=10;
static int velz_error_times=10;
static int deltavelx_error_times=10;
static int deltavely_error_times=10;
static int deltavelz_error_times=10;
static int angvelx_error_times=10;
static int angvely_error_times=10;
static int angvelz_error_times=10;

static int posx_recover_times=100;
static int posy_recover_times=10;
static int posz_recover_times=100;
static int roll_recover_times=100;
static int pitch_recover_times=100;
static int yaw_recover_times=100;
static int velx_recover_times=10;
static int vely_recover_times=10;
static int velz_recover_times=10;
static int deltavelx_recover_times=10;
static int deltavely_recover_times=10;
static int deltavelz_recover_times=10;
static int angvelx_recover_times=100;
static int angvely_recover_times=100;
static int angvelz_recover_times=100;

void RECOVER::detect_and_recover(float time, float target, float real, float &predict_data, float &recover_data, bool &recover_flag, std::string type, 
float& ml_err_sum, float& ml_max_mse, int& w_index, int window, float threshold_err, float threshold_rec, int& err_add, int err_times, int& rec_add, int rec_times){

    if(!is_armed){
        w_index=0;
        ml_err_sum=0;
        err_add=0;
        rec_add=0;
        predict_data = real;
        return ;
    }

    //这里是加速度部分，没有神经网络预测，只是通过速度计算加速度，有大部分重复代码
    if(type=="deltavelx"||type=="deltavely"||type=="deltavelz"){
        float p = 0;
        if(type=="deltavelx"){
            p=velx;
            predict_data = p;
        }
        if(type=="deltavely"){
            p=vely;
            predict_data = p;
        }
        if(type=="deltavelz"){
            p=velz;
            predict_data = p;
        }        
        if(bigger10 == false){
            if (static_cast<int>(previousValues.size()) == numberOfIterations)
            {
                bigger10 = true;
            }
        }
        float accel = 0;
        if (bigger10 == true) {
            // 当list满时，通过速度计算加速度，并删除第一个元素
            accel = (p - previousValues.front()) / (time * numberOfIterations);
            previousValues.pop_front();
        }

        // 将当前循环的数值插入到list尾部
        previousValues.push_back(p);
        if (bigger10 == true) {
            p = accel;
        }

        // float phi = 3.141592653589731;
        // float alpha = 0.025/(0.025+0.025/(2*phi*50));
        // p = alpha*p+(1-alpha)*recover_data;
        recover_data = p/-1000;

        float error=cal_error(p,real);
        ml_err_sum+=error*error;
        float mse=ml_err_sum/(w_index+1);
        // printf("mse=%f",mse);
        // printf("target=%f\n",target);
        // printf("recover=%f\n",p);

        if(mse>ml_max_mse){
            ml_max_mse=mse;
            // printf("max_error=%f,,,\n",mse);
        }

        std::fstream fp;
        fp.open("/home/zzx/UAV/data/threshold/"+type+"_threshold.csv", std::ios::app);
        fp << mse << std::endl;
        fp.close();

        w_index++;
        if(w_index>=window){
            w_index=0;
            ml_err_sum=0;
            err_add=0;
            rec_add=0;
        }

        if(mse>threshold_err){
            err_add++;
        }
        // printf("%s,,,mse=%f,threshold=%d\n",type.c_str(),mse,threshold_err);
        // if(err_add>err_times) recoverBOOL=true;
        if(err_add>err_times) recover_flag=true;  

        if(recover_flag){
            if(mse<threshold_rec) rec_add++;
            if(rec_add>rec_times){
                ml_err_sum=0;
                err_add=0;
                rec_add=0;
                recover_flag=false;
            }
        }
        return ;
    }

    float p=predict(target,predict_data,type);
    predict_data = p;
    if(type=="posx"||type=="posy"||type=="posz"||type=="velx"||type=="vely"||type=="velz"){
        p/=100;
    }
    if(type=="angvelx"||type=="angvely"||type=="angvelz"){
        p/=1000;
    }
    if(type=="roll"){
        if(p>1600)
            p=1600;
        if(p<-2700)
            p=-2700;
    }
    recover_data = p;

    float error=cal_error(p,real);
    ml_err_sum+=error*error;
    float mse=ml_err_sum/(w_index+1);
    // printf("mse=%f",mse);
    // printf("target=%f\n",target);
    // printf("recover=%f\n",p);

    if(mse>ml_max_mse){
        ml_max_mse=mse;
        // printf("max_error=%f,,,\n",mse);
    }

    //收集各个数据在正常情况下的mse，用于设置检测攻击的阈值
    std::fstream fp;
    fp.open("/home/zzx/UAV/data/threshold/"+type+"_threshold.csv", std::ios::app);
    fp << mse << std::endl;
    fp.close();

    w_index++;
    if(w_index>=window){
        w_index=0;
        ml_err_sum=0;
        err_add=0;
        rec_add=0;
    }

    if(mse>threshold_err){
        err_add++;
    }

    if(err_add>err_times) recover_flag=true;  

    if(recover_flag){
        if(mse<threshold_rec) rec_add++;
        if(rec_add>rec_times){
            ml_err_sum=0;
            err_add=0;
            rec_add=0;
            recover_flag=false;
        }
    }
}

void RECOVER::posxrecoevery(float real)
{
    detect_and_recover(0, targetpos.x, real, MLposx, posx, recover_posx, "posx", ml_posx_err_sum, ml_posx_max_mse, ml_posx_w_index, ml_posx_window, ml_posx_threshold, ml_posx_threshold_rec, posx_error_add, posx_error_times, posx_recover_add, posx_recover_times);
};
void RECOVER::posyrecoevery(float real)
{
    detect_and_recover(0, targetpos.y, real, MLposy, posy, recover_posy, "posy", ml_posy_err_sum, ml_posy_max_mse, ml_posy_w_index, ml_posy_window, ml_posy_threshold, ml_posy_threshold_rec, posy_error_add, posy_error_times, posy_recover_add, posy_recover_times);
};
void RECOVER::poszrecoevery(float real)
{
    detect_and_recover(0, targetpos.z, real, MLposz, posz, recover_posz, "posz", ml_posz_err_sum, ml_posz_max_mse, ml_posz_w_index, ml_posz_window, ml_posz_threshold, ml_posz_threshold_rec, posz_error_add, posz_error_times, posz_recover_add, posz_recover_times);
};
void RECOVER::rollrecoevery(float real)
{
    detect_and_recover(0, targetattitude.x, real, MLroll, roll, recover_roll, "roll", ml_roll_err_sum, ml_roll_max_mse, ml_roll_w_index, ml_roll_window, ml_roll_threshold, ml_roll_threshold_rec, roll_error_add, roll_error_times, roll_recover_add, roll_recover_times);
};
void RECOVER::pitchrecoevery(float real)
{
    detect_and_recover(0, targetattitude.y, real, MLpitch, pitch, recover_pitch, "pitch", ml_pitch_err_sum, ml_pitch_max_mse, ml_pitch_w_index, ml_pitch_window, ml_pitch_threshold, ml_pitch_threshold_rec, pitch_error_add, pitch_error_times, pitch_recover_add, pitch_recover_times);
};
void RECOVER::velxrecoevery(float real)
{
    detect_and_recover(0, targetvel.x, real, MLvelx, velx, recover_velx, "velx", ml_velx_err_sum, ml_velx_max_mse, ml_velx_w_index, ml_velx_window, ml_velx_threshold, ml_velx_threshold_rec, velx_error_add, velx_error_times, velx_recover_add, velx_recover_times);
};
void RECOVER::velyrecoevery(float real)
{
    detect_and_recover(0, targetvel.y, real, MLvely, vely, recover_vely, "vely", ml_vely_err_sum, ml_vely_max_mse, ml_vely_w_index, ml_vely_window, ml_vely_threshold, ml_vely_threshold_rec, vely_error_add, vely_error_times, vely_recover_add, vely_recover_times);
};
void RECOVER::velzrecoevery(float real)
{
    detect_and_recover(0, targetvel.z, real, MLvelz, velz, recover_velz, "velz", ml_velz_err_sum, ml_velz_max_mse, ml_velz_w_index, ml_velz_window, ml_velz_threshold, ml_velz_threshold_rec, velz_error_add, velz_error_times, velz_recover_add, velz_recover_times);
};
void RECOVER::deltavelxrecoevery(float time, float real)
{
    detect_and_recover(time, 0, real, MLdeltavelx, deltavelx, recover_deltavelx, "deltavelx", ml_deltavelx_err_sum, ml_deltavelx_max_mse, ml_deltavelx_w_index, ml_deltavelx_window, ml_deltavelx_threshold, ml_deltavelx_threshold_rec, deltavelx_error_add, deltavelx_error_times, deltavelx_recover_add, deltavelx_recover_times);
};
void RECOVER::deltavelyrecoevery(float time, float real)
{
    detect_and_recover(time, 0, real, MLdeltavely, deltavely, recover_deltavely, "deltavely", ml_deltavely_err_sum, ml_deltavely_max_mse, ml_deltavely_w_index, ml_deltavely_window, ml_deltavely_threshold, ml_deltavely_threshold_rec, deltavely_error_add, deltavely_error_times, deltavely_recover_add, deltavely_recover_times);
};
void RECOVER::deltavelzrecoevery(float time, float real)
{
    detect_and_recover(time, 0, real, MLdeltavelz, deltavelz, recover_deltavelz, "deltavelz", ml_deltavelz_err_sum, ml_deltavelz_max_mse, ml_deltavelz_w_index, ml_deltavelz_window, ml_deltavelz_threshold, ml_deltavelz_threshold_rec, deltavelz_error_add, deltavelz_error_times, deltavelz_recover_add, deltavelz_recover_times);
};
void RECOVER::angvelxrecoevery(float real)
{
    detect_and_recover(0, targetangvel.x, real, MLangvelx, angvelx, recover_angvelx, "angvelx", ml_angvelx_err_sum, ml_angvelx_max_mse, ml_angvelx_w_index, ml_angvelx_window, ml_angvelx_threshold, ml_angvelx_threshold_rec, angvelx_error_add, angvelx_error_times, angvelx_recover_add, angvelx_recover_times);
};
void RECOVER::angvelyrecoevery(float real)
{
    detect_and_recover(0, targetangvel.y, real, MLangvely, angvely, recover_angvely, "angvely", ml_angvely_err_sum, ml_angvely_max_mse, ml_angvely_w_index, ml_angvely_window, ml_angvely_threshold, ml_angvely_threshold_rec, angvely_error_add, angvely_error_times, angvely_recover_add, angvely_recover_times);
};
void RECOVER::angvelzrecoevery(float real)
{
    detect_and_recover(0, targetangvel.z, real, MLangvelz, angvelz, recover_angvelz, "angvelz", ml_angvelz_err_sum, ml_angvelz_max_mse, ml_angvelz_w_index, ml_angvelz_window, ml_angvelz_threshold, ml_angvelz_threshold_rec, angvelz_error_add, angvelz_error_times, angvelz_recover_add, angvelz_recover_times);
};

static float to_magyaw(float p)
{
    p/=100;
    if(360-p<180) p-=360;
    p=p*3.141592653589731/180;
    return p;
}
void RECOVER::yawrecoevery(float real)
{
    if(!is_armed){
        MLyaw=real;
    }
    float p=predict(targetattitude.z,real,"yaw");

    int l =36000;
    int e1 = (l-int(MLyaw)+int(real))%l;
    int e2 = (l+int(MLyaw)-int(real))%l;
    int e = e1<e2?e1:e2;
    float error = e;

    ml_yaw_err_sum+=error*error;
    float mse=ml_yaw_err_sum/(ml_yaw_w_index+1);

    if(mse>ml_yaw_max_mse){
        ml_yaw_max_mse=mse;
    }

    //收集yaw在正常情况下的mse，用于设置检测攻击的阈值
    std::fstream fp;
    fp.open("/home/zzx/UAV/data/threshold/yaw_threshold.csv", std::ios::app);
    fp << mse << std::endl;
    fp.close();

    ml_yaw_w_index++;
    if(ml_yaw_w_index>=ml_yaw_window){
        ml_yaw_w_index=0;
        ml_yaw_err_sum=0;
        yaw_error_add=0;
        yaw_recover_add=0;
    }

    
    if(mse>ml_yaw_threshold) yaw_error_add++;
    if(yaw_error_add>yaw_error_times) recover_yaw=true;

    yaw=to_magyaw(p);

    if(recover_yaw){
        if(mse<ml_yaw_threshold_rec) yaw_recover_add++;
        if(yaw_recover_add>yaw_recover_times){
            ml_yaw_err_sum=0;
            yaw_error_add=0;
            yaw_recover_add=0;
            recover_yaw=false;
        }
    }
};
