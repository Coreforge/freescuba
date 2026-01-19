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
            t rootNeighbourCorrection1[2] = {0., 0.};
            t rootNeighbourCorrection2[2] = {0., 0.};
        };

        FingerCalibration index;
        FingerCalibration middle;
        FingerCalibration ring;
        FingerCalibration pinky;
        FingerCalibration thumb;
    };

    struct GloveValues{
        struct FingerValues{
            t root1 = (t)0.;
            t root2 = (t)0.;
            t tip = (t)0.;
        };
        FingerValues index;
        FingerValues middle;
        FingerValues ring;
        FingerValues pinky;
        FingerValues thumb;
        t thumbBase = (t)0.;
    };

    struct ModelOutputs{
        struct FingerOutputs{
            t root = (t)0.;
            t tip = (t)0.;
            t splay = (t)0.;
        };

        FingerOutputs index;
        FingerOutputs middle;
        FingerOutputs ring;
        FingerOutputs pinky;
        FingerOutputs thumb;
        t thumbBase = (t)0.;
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

#define APPLY_FINGER_BASE(finger, corr1, corr2) \
        outputs.finger.root = ((GET_SINGLE_SENSOR(finger, root1) + \
            ((corr1 + calibration.finger.rootNeighbourCorrection1[1]) * (calibration.finger.rootNeighbourCorrection1[0])))\
         + (GET_SINGLE_SENSOR(finger, root2) + \
            ((corr2 + calibration.finger.rootNeighbourCorrection2[1]) * (calibration.finger.rootNeighbourCorrection2[0])))\
        ) / 2.;
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

        //APPLY_FINGER_BASE(index, GET_SINGLE_SENSOR(middle, root2), GET_SINGLE_SENSOR(thumb, root1))
        //APPLY_FINGER_BASE(middle, GET_SINGLE_SENSOR(ring, root2), GET_SINGLE_SENSOR(index, root1))
        //APPLY_FINGER_BASE(ring, GET_SINGLE_SENSOR(pinky, root2), GET_SINGLE_SENSOR(middle, root1))
        //APPLY_FINGER_BASE(pinky, 0., GET_SINGLE_SENSOR(ring, root1))
        //APPLY_FINGER_BASE(thumb, GET_SINGLE_SENSOR(index, root2), 0.)

        APPLY_FINGER_BASE(index, 0., 0.)
        APPLY_FINGER_BASE(middle, 0., 0.)
        APPLY_FINGER_BASE(ring, 0., 0.)
        APPLY_FINGER_BASE(pinky, 0., 0.)
        APPLY_FINGER_BASE(thumb, 0., 0.)

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
