#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPANGVELX
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPANGVELX()
    {
        weights1 << 0.9143419861793518f, 0.3661559522151947f, -0.30912527441978455f, 1.0700440406799316f;
        weights2 << 1.393375039100647f, -0.014524165540933609f;

        bias1 << 0.0369613952934742f, -0.03220343589782715f;
        bias2 << 0.036822594702243805f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
