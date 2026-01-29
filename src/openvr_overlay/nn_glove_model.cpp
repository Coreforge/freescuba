
//#include <cstddef>
#include <af/device.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <flashlight/fl/nn/Init.h>
#include <flashlight/fl/nn/modules/Activations.h>
#include <flashlight/fl/nn/modules/Conv2D.h>
#include <flashlight/fl/nn/modules/Linear.h>
#include <flashlight/fl/optim/AdamOptimizer.h>
#include <flashlight/fl/optim/NAGOptimizer.h>
#include <flashlight/fl/tensor/Init.h>
#include <flashlight/fl/dataset/datasets.h>
#include <flashlight/fl/meter/meters.h>
#include <flashlight/fl/nn/nn.h>
#include <flashlight/fl/optim/optim.h>
#include <flashlight/fl/tensor/Random.h>
#include <flashlight/fl/tensor/TensorBase.h>
#include <flashlight/fl/tensor/TensorAdapter.h>
#include <arrayfire.h>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include "nn_glove_model.h"
#include "glove_model.hpp"
#include "utils.h"

#include <flashlight/fl/flashlight.h>

typedef float NNGloveDataType;
const double valueScale = 1.;

void NNGloveModel::train(std::vector<GloveModel<double>::GloveValues>& inputs,
        std::vector<GloveModel<double>::ModelOutputs>& outputs){

    std::vector<GloveModel<NNGloveDataType>::GloveValues> fInputs;
    std::vector<GloveModel<NNGloveDataType>::ModelOutputs> fOutputs;
    fInputs.reserve(inputs.size());
    fOutputs.reserve(outputs.size());
    for(auto& e : inputs){
        fInputs.emplace_back(e.convert<NNGloveDataType>(valueScale));
    }
    for(auto& e : outputs){
        fOutputs.emplace_back(e.convert<NNGloveDataType>(valueScale));
    }

    fl::Shape ioShape({16,(ssize_t)fInputs.size()});
    auto inTensor = fl::Tensor::fromBuffer(ioShape, (NNGloveDataType*)fInputs.data(), fl::Location::Host);
    auto outTensor = fl::Tensor::fromBuffer(ioShape, (NNGloveDataType*)fOutputs.data(), fl::Location::Host);
    
    //af::info();
    
    fl::TensorDataset trainset(std::vector<fl::Tensor>({inTensor, outTensor}));

    fl::Sequential model;
    model.add(fl::Linear(16, 42));
    model.add(fl::ELU());
    model.add(fl::Linear(42, 42));
    model.add(fl::Tanh());
    model.add(fl::Linear(42, 42));
    model.add(fl::ELU());
    model.add(fl::Linear(42, 16));

    auto loss = fl::MeanSquaredError();

    // these very aggressive settings probably won't work with more training data
    const float learningRate = 0.01;
    const float momentum = 0.99;
    //auto sgd = fl::SGDOptimizer(model.params(), learningRate, momentum);
    //auto sgd = fl::AdamOptimizer(model.params(), learningRate, 0.999, 0.9999);
    auto sgd = fl::NAGOptimizer(model.params(), learningRate, momentum);

    fl::AverageValueMeter meter;

    const size_t epochs = 2000;
    const double min_loss = 5e-12;
    double lastLoss = NAN;
    for(size_t epoch = 0; epoch < epochs; epoch++){
        meter.reset();
        for(auto& sample : trainset){
            sgd.zeroGrad();

            auto result = model(fl::input(sample[0]));

            auto l = loss(result, fl::noGrad(sample[1]));

            l.backward();

            sgd.step();

            meter.add(l.scalar<NNGloveDataType>());
        }
        if(!std::isnan(lastLoss)){
            double lossChange = lastLoss - meter.value()[0];
            //std::cout << "Epoch " << epoch << " Loss change: " << lossChange << std::endl;
        }
        lastLoss = meter.value()[0];
        if(epoch % 20 == 0){
            std::cout << "Epoch " << epoch << " Loss: " << meter.value()[0] << std::endl;
        }
        if(meter.value()[0] < min_loss){
            std::cout << "Minimum loss threshold reached, stopping training" << std::endl;
            break;
        }
    }


    std::cout << "Training done" << std::endl;
    resultModel = model;
    resultModel.eval();
    trained = true;
}

void NNGloveModel::run(const GloveModel<double>::GloveValues& input,
        GloveModel<double>::ModelOutputs& output){

    auto convData = input.convert<NNGloveDataType>(valueScale);
    auto inTensor = fl::Tensor::fromBuffer({16}, (NNGloveDataType*)&convData, fl::Location::Host);
    GloveModel<NNGloveDataType>::ModelOutputs convOutput;
    if(trained){
        auto result = resultModel(fl::input(inTensor));
        auto tens = result.tensor();
        //std::cout << "Out: " << tens << std::endl;
        std::memcpy(&convOutput, tens.host<NNGloveDataType>(), sizeof(convOutput));
        // double conversion, because scaling down on the first one would loose nearly all data
        output = convOutput.convert<double>().convert<double>(1./valueScale);
    }

}