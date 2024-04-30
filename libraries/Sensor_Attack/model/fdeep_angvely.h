#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPANGVELY
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPANGVELY()
    {
        weights1 << 0.9116492867469788f, 0.3676621615886688f, -0.31181737780570984f, 1.071550726890564f;
        weights2 << 1.390478253364563f, -0.017103031277656555f;

        bias1 << 0.034269366413354874f, -0.030698025599122047f;
        bias2 << 0.03416445478796959f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
