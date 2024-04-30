#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPPOSY
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPPOSY()
    {
        weights1 << 0.9494864344596863f, 0.3613154888153076f, -0.272408664226532f, 1.0654454231262207f;
        weights2 << 1.4265165328979492f, 0.0218576081097126f;

        bias1 << -0.003087414661422372f, -0.05396057665348053f;
        bias2 << -0.0018681322690099478f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
