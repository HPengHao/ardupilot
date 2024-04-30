#pragma once
#include <vector>
#include "../include/Eigen/Dense"

class FDEEPVELZ
{
public:
    std::vector<Eigen::MatrixXf> weightss;
    std::vector<Eigen::MatrixXf> biasess;

    Eigen::Matrix<float, 2, 4> weights1;
    Eigen::Matrix<float, 4, 2> weights2;
    Eigen::Matrix<float, 2, 1> weights3;

    Eigen::Matrix<float, 1, 4> bias1;
    Eigen::Matrix<float, 1, 2> bias2;
    Eigen::Matrix<float, 1, 1> bias3;

    FDEEPVELZ()
    {
        weights1 << 0.6809143424034119f, 0.3505195081233978f, -0.2571931481361389f, 0.9128133058547974f, -0.8792787194252014f, 0.3548567593097687f, 0.9915015697479248f, -0.6387470364570618f;
        weights2 << 0.9342005252838135f, -0.035964250564575195f, 0.5678616762161255f, 0.4857454299926758f, 0.6898292303085327f, -0.7626423835754395f, 0.3932209610939026f, 0.058797597885131836f;
        weights3 << 1.0069483518600464f, -0.0653144121170044f;

        bias1 << -0.03639507666230202f, 0.0013185320422053337f, 0.001353056519292295f, -0.035748034715652466f;
        bias2 << 0.002195088192820549f, 0.0f;
        bias3 << 0.002671538619324565f;

        weightss.push_back(weights1);
        weightss.push_back(weights2);
        weightss.push_back(weights3);

        biasess.push_back(bias1);
        biasess.push_back(bias2);
        biasess.push_back(bias3);
    }
};
