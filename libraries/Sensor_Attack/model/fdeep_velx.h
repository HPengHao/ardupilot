#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPVELX
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPVELX()
    {
        weights1 << 0.9322876930236816f, 0.32061856985092163f, -0.25220614671707153f, 1.0528286695480347f;
        weights2 << 1.384903907775879f, 0.034015949815511703f;

        bias1 << 0.001720024854876101f, -0.10259220004081726f;
        bias2 << 0.0028843656182289124f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
