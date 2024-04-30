#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPPOSX
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

    FDEEPPOSX()
    {
        weights1 << 0.8314346671104431f, 0.3523810803890228f, -0.39225491881370544f, 1.0560390949249268f;
        weights2 << 1.1274551153182983f, 0.001869315980002284f, 0.6054357290267944f, 0.6407061815261841f;
        weights3 << 0.9231151938438416f, -0.11045027524232864f;

        bias1 << -0.0428360179066658f, -0.04290611669421196f;
        bias2 << -0.04271838441491127f, 0.03189224377274513f;
        bias3 << -0.03453238680958748f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);
        weightss.push_back(weights3);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
        biasess.push_back(bias3);
    }
};
