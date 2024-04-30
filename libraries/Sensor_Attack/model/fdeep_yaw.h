#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPYAW
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 2> weights1;
    Eigen::Matrix<float, 2, 2> weights2;
    Eigen::Matrix<float, 2, 1> weights3;

    Eigen::Matrix<float, 1, 2> bias1;
    Eigen::Matrix<float, 1, 2> bias2;
    Eigen::Matrix<float, 1, 1> bias3;

    FDEEPYAW()
    {
        weights1 << 0.7885835766792297f, 0.3122911751270294f, -0.3816065490245819f, 1.0680255889892578f;
        weights2 << 1.0638140439987183f, 0.08486302196979523f, 0.6000797748565674f, 0.6530970335006714f;
        weights3 << 0.8845161199569702f, -0.11937345564365387f;

        bias1 << -0.01597796380519867f, -0.01002319436520338f;
        bias2 << -0.0067979274317622185f, -0.02401023358106613f;
        bias3 << 0.020899740979075432f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);
        weightss.push_back(weights3);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
        biasess.push_back(bias3);
    }
};
