#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPPOSZ
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPPOSZ()
    {
        weights1 << 0.9408086538314819f, 0.3690876066684723f, -0.25235700607299805f, 1.0791112184524536f;
        weights2 << 1.4041099548339844f, 0.033245109021663666f;

        bias1 << -5.872859401279129e-05f, -0.030943339690566063f;
        bias2 << 0.0005904047866351902f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
