#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPROLL
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPROLL()
    {
        weights1 << 0.9327749609947205f, 0.31769275665283203f, -0.23624613881111145f, 1.0753759145736694f;
        weights2 << 1.3754184246063232f, 0.04781113192439079f;

        bias1 << -0.000819512119051069f, -0.1056177094578743f;
        bias2 << 0.0002928026660811156f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
