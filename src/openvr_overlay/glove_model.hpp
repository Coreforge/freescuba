#pragma once

#include <vector>

#include "../ipc_protocol.hpp"

// model to transform the glove reading into finger values
// templated for use with ceres


template<typename t>
class GloveModel{
public:
    struct GloveModelCalibration{
        struct JointCalibration{
            t zeroPoint = 1400;
            t factor = 0.002;
        };
        struct FingerCalibration{
            JointCalibration root1;
            JointCalibration root2;
            JointCalibration tip;
            JointCalibration splay;
        };

        FingerCalibration index;
        FingerCalibration middle;
        FingerCalibration ring;
        FingerCalibration pinky;
        FingerCalibration thumb;
    };

    struct GloveValues{
        struct FingerValues{
            t root1;
            t root2;
            t tip;
        };
        FingerValues index;
        FingerValues middle;
        FingerValues ring;
        FingerValues pinky;
        FingerValues thumb;
        t thumbBase;
    };

    struct ModelOutputs{
        struct FingerOutputs{
            t root;
            t tip;
            t splay;
        };

        FingerOutputs index;
        FingerOutputs middle;
        FingerOutputs ring;
        FingerOutputs pinky;
        FingerOutputs thumb;
        t thumbBase;
    };

    struct ModelOutputComponents{
        struct FingerOutputs{
            bool root;
            bool tip;
            bool splay;
        };

        FingerOutputs index;
        FingerOutputs middle;
        FingerOutputs ring;
        FingerOutputs pinky;
        FingerOutputs thumb;
        bool thumbBase;
    };

    template<typename i, typename c, typename o>
    static void apply(const typename GloveModel<i>::GloveValues& inputs, typename GloveModel<c>::GloveModelCalibration& calibration, typename GloveModel<o>::ModelOutputs& outputs){
#define GET_SINGLE_SENSOR(finger, joint) \
        ((inputs.finger.joint - calibration.finger.joint.zeroPoint) * calibration.finger.joint.factor)
#define APPLY_SINGLE_SENSOR(finger, joint) \
        outputs.finger.joint = GET_SINGLE_SENSOR(finger, joint);

#define APPLY_FINGER_BASE(finger) \
        outputs.finger.root = (GET_SINGLE_SENSOR(finger, root1) + GET_SINGLE_SENSOR(finger, root2)) / 2.;
        //outputs.finger.root = ((inputs.finger.root1 + inputs.finger.root2) - (calibration.finger.root1.zeroPoint + calibration.finger.root2.zeroPoint)) \
        //    * (calibration.finger.root1.factor) / 2.;
        // (GET_SINGLE_SENSOR(finger, root1) + GET_SINGLE_SENSOR(finger, root2)) / 2.;
        
#define APPLY_FINGER_SPLAY(finger) \
        outputs.finger.splay = ((inputs.finger.root1 - inputs.finger.root2) - calibration.finger.splay.zeroPoint) \
            * calibration.finger.splay.factor;

        APPLY_SINGLE_SENSOR(index, tip)
        APPLY_SINGLE_SENSOR(middle, tip)
        APPLY_SINGLE_SENSOR(ring, tip)
        APPLY_SINGLE_SENSOR(pinky, tip)
        APPLY_SINGLE_SENSOR(thumb, tip)

        APPLY_FINGER_BASE(index)
        APPLY_FINGER_BASE(middle)
        APPLY_FINGER_BASE(ring)
        APPLY_FINGER_BASE(pinky)
        APPLY_FINGER_BASE(thumb)

        APPLY_FINGER_SPLAY(index)
        APPLY_FINGER_SPLAY(middle)
        APPLY_FINGER_SPLAY(ring)
        APPLY_FINGER_SPLAY(pinky)
        APPLY_FINGER_SPLAY(thumb)
        
    }
};

// separate non-templated class
class GloveModelSolver{
public:
    void calibrate(const protocol::ContactGloveState_t::HandFingersCalibrationData_t& calibrationData);
    void apply(protocol::ContactGloveState_t& state);

private:
    size_t convertCalibrationData(const protocol::ContactGloveState_t::HandFingersCalibrationData_t& calibrationData,
        std::vector<GloveModel<double>::GloveValues>& inputs, std::vector<GloveModel<double>::ModelOutputs>& outputs,
        std::vector<GloveModel<double>::ModelOutputComponents>& components);
    GloveModel<double>::GloveModelCalibration modelCalibration;
};
