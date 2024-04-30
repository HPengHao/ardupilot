#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPPITCH
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPPITCH()
    {
        weights1 << 0.9325962662696838f, 0.3608437180519104f, -0.2909286320209503f, 1.064721941947937f;
        weights2 << 1.4131437540054321f, 0.0029223596211522818f;

        bias1 << 0.07508006691932678f, -0.043358754366636276f;
        bias2 << 0.07427630573511124f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
