#pragma once

#include "glove_model.hpp"
#include <vector>

#include <flashlight/fl/flashlight.h>

class NNGloveModel{
public:
    void train(std::vector<GloveModel<double>::GloveValues>& inputs,
        std::vector<GloveModel<double>::ModelOutputs>& outputs);

    void run(const GloveModel<double>::GloveValues& input,
        GloveModel<double>::ModelOutputs& output);

private:
    fl::Sequential resultModel;
    bool trained = false;
};