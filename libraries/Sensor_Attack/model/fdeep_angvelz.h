#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPANGVELZ
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 4> weights1;
    Eigen::Matrix<float, 4, 1> weights2;

    Eigen::Matrix<float, 1, 4> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPANGVELZ()
    {
        weights1 << 0.7163782119750977f, 0.29470348358154297f, -0.22165440022945404f, 0.9611033201217651f, -0.8438777923583984f, 0.27411121129989624f, 1.001926302909851f, -0.6030513048171997f;
        weights2 << 1.0497779846191406f, 0.016657738015055656f, 0.6467181444168091f, 0.6013417840003967f;

        bias1 << 0.0f, -0.03055529296398163f, 0.06092681363224983f, 0.061126016080379486f;
        bias2 << 0.059646304696798325f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
