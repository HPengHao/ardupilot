#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPVELY
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 1> weights2;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 1> bias2;

    FDEEPVELY()
    {
        weights1 << 0.9225372672080994f, 0.30777794122695923f, -0.2532462179660797f, 1.043232798576355f;
        weights2 << 1.3717517852783203f, 0.032133493572473526f;

        bias1 << 0.00816019531339407f, -0.09653420746326447f;
        bias2 << 0.009000384248793125f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
    }
};
