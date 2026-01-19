#include "glove_model.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <cstdint>
#include <iostream>
#include <vector>

#include "utils.h"

// currently, it's assumed that the structs this uses and arrays ceres views them as are packed the same.
// If this turns out to not be reliably the case, conversion between struct and array has to be done manually (which is cleaner, but more effort)

// very simple. Apply the model, then return the differences between the output values and the expected values as the residuals
// masking off values should be possible as well, allowing for certain calibration poses to not affect everything
struct GloveCostFunctor{
    GloveCostFunctor(const GloveModel<double>::ModelOutputs& outputs,
        const GloveModel<double>::GloveValues inputs,
        GloveModel<double>::ModelOutputComponents components) : 
        outputs(outputs),
        inputs(inputs),
        components(components){

    }

    template<typename reftype, typename type>
    void calcResiduals(const typename GloveModel<reftype>::ModelOutputs& ref,
        const typename GloveModel<type>::ModelOutputs& result,
        typename GloveModel<type>::ModelOutputs& residual) const{

#define FINGER_RESIDUAL(finger) \
        residual.finger.root = (components.finger.root) ? (result.finger.root - ref.finger.root) : (type)0.; \
        residual.finger.tip = (components.finger.tip) ? (result.finger.tip - ref.finger.tip) : (type)0.; \
        residual.finger.splay = components.finger.splay ? (result.finger.splay - ref.finger.splay) : (type)0.;

        FINGER_RESIDUAL(index)
        FINGER_RESIDUAL(middle)
        FINGER_RESIDUAL(ring)
        FINGER_RESIDUAL(pinky)
        FINGER_RESIDUAL(thumb)

        residual.thumbBase = result.thumbBase - ref.thumbBase;
        
    }

    template<typename type>
    bool operator()(const type* calibrationArr, type* residualsArr) const{
        typename GloveModel<type>::GloveModelCalibration* calibration = (typename GloveModel<type>::GloveModelCalibration*)calibrationArr;
        typename GloveModel<type>::ModelOutputs* residuals = (typename GloveModel<type>::ModelOutputs*) residualsArr;
        typename GloveModel<type>::ModelOutputs result;

        GloveModel<type>::template apply<double, type, type>(inputs,
            *calibration, result);

        calcResiduals<double, type>(outputs, result, *residuals);
        
        return true;
    }

    
    static ceres::CostFunction* Create(const GloveModel<double>::ModelOutputs& outputs, 
        const GloveModel<double>::GloveValues& inputs,
        GloveModel<double>::ModelOutputComponents components){
            return new ceres::AutoDiffCostFunction<GloveCostFunctor,
                sizeof(GloveModel<uint8_t>::ModelOutputs), 
                sizeof(GloveModel<uint8_t>::GloveModelCalibration)>(
                    new GloveCostFunctor(outputs, inputs, components)
                );
        }

    GloveModel<double>::ModelOutputs outputs;
    GloveModel<double>::GloveValues inputs;
    GloveModel<double>::ModelOutputComponents components;
};

void GloveModelSolver::calibrate(const protocol::ContactGloveState_t::HandFingersCalibrationData_t& calibrationData){
    ceres::Problem problem;
    std::vector<GloveModel<double>::GloveValues> inputs;
    std::vector<GloveModel<double>::ModelOutputs> outputs;
    std::vector<GloveModel<double>::ModelOutputComponents> components;

    convertCalibrationData(calibrationData, inputs, outputs, components);

    // inputs and outputs need to have the same length, which they should
    for(size_t i = 0; i < inputs.size(); i++){
        // the cast to double* assumes same packing tightness as an array
        std::cout << "Add glove pose" << std::endl;
        problem.AddResidualBlock(GloveCostFunctor::Create(outputs[i], inputs[i], components[i]), 
            nullptr, (double*)&modelCalibration);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    std::cout << "GloveModel calibratrion done" << std::endl;

    #define PRINT_CALIB_FINGER_PARAM(finger, param) \
    std::cout << #finger" "#param" " << modelCalibration.finger.param.zeroPoint << " " << modelCalibration.finger.param.factor << std::endl;
    #define PRINT_CALIB_FINGER_CORR(finger) \
    std::cout << #finger " neighbour correction " << modelCalibration.finger.rootNeighbourCorrection1[0] << \
        " " << modelCalibration.finger.rootNeighbourCorrection1[1] << \
        " " << modelCalibration.finger.rootNeighbourCorrection2[0] << \
        " " << modelCalibration.finger.rootNeighbourCorrection2[1] << std::endl;

    #define PRINT_CALIB_FINGER(finger, dummy) \
    PRINT_CALIB_FINGER_PARAM(finger, root1) \
    PRINT_CALIB_FINGER_PARAM(finger, root2) \
    PRINT_CALIB_FINGER_PARAM(finger, tip) \
    PRINT_CALIB_FINGER_PARAM(finger, splay) \
    PRINT_CALIB_FINGER_CORR(finger)

    FOREACH_FINGER(PRINT_CALIB_FINGER)
}

void GloveModelSolver::apply(protocol::ContactGloveState_t& state){
    GloveModel<double>::GloveValues inputs;
    GloveModel<double>::ModelOutputs outputs;
    #define COPY_FINGER(finger, dummy) \
    inputs.finger.root1 = state.finger##Root1Raw; \
    inputs.finger.root2 = state.finger##Root2Raw; \
    inputs.finger.tip = state.finger##TipRaw;

    FOREACH_FINGER(COPY_FINGER)

    GloveModel<double>::apply<double, double, double>(inputs, modelCalibration, outputs);

    #define COPY_FINGER_OUTPUT(finger, dummy) \
    state.finger##Root = outputs.finger.root; \
    state.finger##Tip = outputs.finger.tip; \
    state.finger##Splay = outputs.finger.splay;

    FOREACH_FINGER(COPY_FINGER_OUTPUT)

    #undef COPY_FINGER
    #undef COPY_FINGER_OUTPUT

}

// this has a horrible amount of macros to reduce duplicate code
// returns the number of poses added
size_t GloveModelSolver::convertCalibrationData(const protocol::ContactGloveState_t::HandFingersCalibrationData_t& calibrationData,
        std::vector<GloveModel<double>::GloveValues>& inputs, std::vector<GloveModel<double>::ModelOutputs>& outputs,
        std::vector<GloveModel<double>::ModelOutputComponents>& components){

    size_t numPoses = 0;

    #define GLOVE_STATE_FIELDS(state) \
    GloveModel<double>::GloveValues state##_inputs; \
    GloveModel<double>::ModelOutputs state##_outputs; \
    GloveModel<double>::ModelOutputComponents state##_components;

    #define GLOVE_APPEND_STATE_FIELDS(state)\
    inputs.emplace_back(state##_inputs); \
    outputs.emplace_back(state##_outputs); \
    components.emplace_back(state##_components); \
    numPoses++;

    #define INPUT_JOINT_STATE(state, finger, sensor, joint) \
    state##_inputs.finger.sensor = calibrationData.finger.joint.state;

    #define INPUT_FINGER_STATE(finger, state)\
    INPUT_JOINT_STATE(state, finger, root1, proximal) \
    INPUT_JOINT_STATE(state, finger, root2, proximal2) \
    INPUT_JOINT_STATE(state, finger, tip, distal)

    // just makes it easier to use with other macros
    #define OUTPUT_FINGER(finger, state, joint, value) \
    state##_outputs.finger.joint = value;
    #define OUTPUT_FINGER_COMPONENT(finger, state, joint, value) \
    state##_components.finger.joint = value;

    // macros done, now set up all the hand states

    GLOVE_STATE_FIELDS(rest)
    GLOVE_STATE_FIELDS(close)
    GLOVE_STATE_FIELDS(splayed)
    GLOVE_STATE_FIELDS(horns)
    GLOVE_STATE_FIELDS(peace)
    GLOVE_STATE_FIELDS(flipoff)
    GLOVE_STATE_FIELDS(point)

    FOREACH_FINGER(INPUT_FINGER_STATE, rest)
    FOREACH_FINGER(OUTPUT_FINGER, rest, root, 0)
    FOREACH_FINGER(OUTPUT_FINGER, rest, tip, 0)
    FOREACH_FINGER(OUTPUT_FINGER, rest, splay, 0)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, rest, root, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, rest, tip, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, rest, splay, true)

    FOREACH_FINGER(INPUT_FINGER_STATE, close)
    FOREACH_FINGER(OUTPUT_FINGER, close, root, 1)
    FOREACH_FINGER(OUTPUT_FINGER, close, tip, 1)
    FOREACH_FINGER(OUTPUT_FINGER, close, splay, 0)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, close, root, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, close, tip, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, close, splay, true)

    FOREACH_FINGER(INPUT_FINGER_STATE, splayed)
    FOREACH_FINGER(OUTPUT_FINGER, splayed, root, 0)
    FOREACH_FINGER(OUTPUT_FINGER, splayed, tip, 0)
    FOREACH_FINGER(OUTPUT_FINGER, splayed, splay, 1)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, splayed, root, false)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, splayed, tip, false)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, splayed, splay, true)

    FOREACH_FINGER(INPUT_FINGER_STATE, horns)
    FOREACH_FINGER(OUTPUT_FINGER, horns, root, 1)
    FOREACH_FINGER(OUTPUT_FINGER, horns, tip, 1)
    OUTPUT_FINGER(thumb, horns, tip, 0)
    OUTPUT_FINGER(index, horns, root, 0)
    OUTPUT_FINGER(index, horns, tip, 0)
    OUTPUT_FINGER(pinky, horns, root, 0)
    OUTPUT_FINGER(pinky, horns, tip, 0)
    FOREACH_FINGER(OUTPUT_FINGER, horns, splay, 0)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, horns, root, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, horns, tip, false)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, horns, splay, true)

    FOREACH_FINGER(INPUT_FINGER_STATE, peace)
    FOREACH_FINGER(OUTPUT_FINGER, peace, root, 1)
    FOREACH_FINGER(OUTPUT_FINGER, peace, tip, 1)
    OUTPUT_FINGER(thumb, peace, tip, 0)
    OUTPUT_FINGER(index, peace, root, 0)
    OUTPUT_FINGER(index, peace, tip, 0)
    OUTPUT_FINGER(middle, peace, root, 0)
    OUTPUT_FINGER(middle, peace, tip, 0)
    FOREACH_FINGER(OUTPUT_FINGER, peace, splay, 0)
    OUTPUT_FINGER(index, peace, splay, 1)
    OUTPUT_FINGER(middle, peace, splay, -1)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, peace, root, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, peace, tip, false)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, peace, splay, true)

    FOREACH_FINGER(INPUT_FINGER_STATE, flipoff)
    FOREACH_FINGER(OUTPUT_FINGER, flipoff, root, 1)
    FOREACH_FINGER(OUTPUT_FINGER, flipoff, tip, 1)
    OUTPUT_FINGER(thumb, flipoff, tip, 0)
    OUTPUT_FINGER(middle, flipoff, root, 0)
    OUTPUT_FINGER(middle, flipoff, tip, 0)
    FOREACH_FINGER(OUTPUT_FINGER, flipoff, splay, 0)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, flipoff, root, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, flipoff, tip, false)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, flipoff, splay, true)

    FOREACH_FINGER(INPUT_FINGER_STATE, point)
    FOREACH_FINGER(OUTPUT_FINGER, point, root, 1)
    FOREACH_FINGER(OUTPUT_FINGER, point, tip, 1)
    OUTPUT_FINGER(thumb, horns, tip, 0)
    OUTPUT_FINGER(index, horns, root, 0)
    OUTPUT_FINGER(index, horns, tip, 0)
    FOREACH_FINGER(OUTPUT_FINGER, point, splay, 0)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, point, root, true)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, point, tip, false)
    FOREACH_FINGER(OUTPUT_FINGER_COMPONENT, point, splay, true)

    // add hand states to vectors
    GLOVE_APPEND_STATE_FIELDS(rest)
    GLOVE_APPEND_STATE_FIELDS(close)
    GLOVE_APPEND_STATE_FIELDS(splayed)
    GLOVE_APPEND_STATE_FIELDS(horns)
    GLOVE_APPEND_STATE_FIELDS(peace)
    GLOVE_APPEND_STATE_FIELDS(flipoff)
    GLOVE_APPEND_STATE_FIELDS(point)

    return numPoses;
}
