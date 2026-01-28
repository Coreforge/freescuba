#pragma once

#include <vector>

#include "../ipc_protocol.hpp"

// model to transform the glove reading into finger values
// templated for use with ceres

class NNGloveModel;
struct RecordedCalibrationPose;

template<typename t>
class GloveModel{
public:
#pragma pack(push, 1)
    struct GloveModelCalibration{
        struct JointCalibration{
            t zeroPoint = 0.3;
            t factor = 1;
        };
        struct FingerCalibration{
            JointCalibration root1;
            JointCalibration root2;
            JointCalibration tip;
            JointCalibration splay;
            t rootNeighbourCorrection1[2] = {0.1, 0.};
            t rootNeighbourCorrection2[2] = {0.1, 0.};
        };

        FingerCalibration index;
        FingerCalibration middle;
        FingerCalibration ring;
        FingerCalibration pinky;
        FingerCalibration thumb;
        JointCalibration thumbBase;
    };

    struct GloveValues{
        struct FingerValues{
            t root1 = (t)0.;
            t root2 = (t)0.;
            t tip = (t)0.;
            // I've found no way to have the compiler pick up conversion or assignment operators correctly, or constructors
            template <typename o>
            typename GloveModel<o>::GloveValues::FingerValues convert(t factor = (t)1.) const{
                typename GloveModel<o>::GloveValues::FingerValues n;
                n.root1 = (o)(root1 * factor);
                n.root2 = (o)(root2 * factor);
                n.tip = (o)(tip * factor);
                return n;
            }
        };
        FingerValues index;
        FingerValues middle;
        FingerValues ring;
        FingerValues pinky;
        FingerValues thumb;
        t thumbBase = (t)0.;

        template <typename o>
        typename GloveModel<o>::GloveValues convert(t factor = (t)1.) const{
            typename GloveModel<o>::GloveValues n;
            n.index = index.template convert<o>(factor);
            n.middle = middle.template convert<o>(factor);
            n.ring = ring.template convert<o>(factor);
            n.pinky = pinky.template convert<o>(factor);
            n.thumb = thumb.template convert<o>(factor);
            n.thumbBase = (o)(thumbBase * factor);
            return n;
        }

    };

    struct ModelOutputs{
        struct FingerOutputs{
            t root = (t)0.;
            t tip = (t)0.;
            t splay = (t)0.;

            template <typename o>
            typename GloveModel<o>::ModelOutputs::FingerOutputs convert(t factor = (t)1.) const{
                typename GloveModel<o>::ModelOutputs::FingerOutputs n;
                n.root = (o)(root * factor);
                n.tip = (o)(tip * factor);
                n.splay = (o)(splay * factor);
                return n;
            }
        };

        FingerOutputs index;
        FingerOutputs middle;
        FingerOutputs ring;
        FingerOutputs pinky;
        FingerOutputs thumb;
        t thumbBase = (t)0.;

        template <typename o>
        typename GloveModel<o>::ModelOutputs convert(t factor = (t)1.) const{
            typename GloveModel<o>::ModelOutputs n;
            n.index = index.template convert<o>(factor);
            n.middle = middle.template convert<o>(factor);
            n.ring = ring.template convert<o>(factor);
            n.pinky = pinky.template convert<o>(factor);
            n.thumb = thumb.template convert<o>(factor);
            n.thumbBase = (o)(thumbBase * factor);
            return n;
        }
    };

    struct ModelOutputComponents{
        struct FingerOutputs{
            bool root = true;
            bool tip = true;
            bool splay = true;
        };

        FingerOutputs index;
        FingerOutputs middle;
        FingerOutputs ring;
        FingerOutputs pinky;
        FingerOutputs thumb;
        bool thumbBase = true;
    };
#pragma pack(pop)
    template<typename i, typename c, typename o>
    static void apply(const typename GloveModel<i>::GloveValues& inputs, typename GloveModel<c>::GloveModelCalibration& calibration, typename GloveModel<o>::ModelOutputs& outputs){
#define GET_SINGLE_SENSOR(finger, joint) \
        ((inputs.finger.joint - calibration.finger.joint.zeroPoint) * calibration.finger.joint.factor)
#define APPLY_SINGLE_SENSOR(finger, joint) \
        outputs.finger.joint = GET_SINGLE_SENSOR(finger, joint);

#define APPLY_FINGER_BASE(finger, corr1, corr2) \
        outputs.finger.root = ((GET_SINGLE_SENSOR(finger, root1) + \
            ((GET_SINGLE_SENSOR(finger, root1) - corr1 /*+ calibration.finger.rootNeighbourCorrection1[1]*/) \
                * (calibration.finger.rootNeighbourCorrection1[0])))\
         + (GET_SINGLE_SENSOR(finger, root2) + \
            ((GET_SINGLE_SENSOR(finger, root2) - corr2 /*+ calibration.finger.rootNeighbourCorrection2[1]*/) \
                * (calibration.finger.rootNeighbourCorrection2[0])))\
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

        outputs.thumbBase =  (inputs.thumbBase - calibration.thumbBase.zeroPoint) * calibration.thumbBase.factor;

        APPLY_FINGER_BASE(index, GET_SINGLE_SENSOR(middle, root2), GET_SINGLE_SENSOR(thumb, root1))
        APPLY_FINGER_BASE(middle, GET_SINGLE_SENSOR(ring, root2), GET_SINGLE_SENSOR(index, root1))
        APPLY_FINGER_BASE(ring, GET_SINGLE_SENSOR(pinky, root2), GET_SINGLE_SENSOR(middle, root1))
        APPLY_FINGER_BASE(pinky, 0., GET_SINGLE_SENSOR(ring, root1))
        APPLY_FINGER_BASE(thumb, GET_SINGLE_SENSOR(index, root2), 0.)

        //APPLY_FINGER_BASE(index, 0., 0.)
        //APPLY_FINGER_BASE(middle, 0., 0.)
        //APPLY_FINGER_BASE(ring, 0., 0.)
        //APPLY_FINGER_BASE(pinky, 0., 0.)
        //APPLY_FINGER_BASE(thumb, 0., 0.)

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
    void calibrate(const protocol::ContactGloveState_t::HandFingersCalibrationData_t& calibrationData,
        std::vector<RecordedCalibrationPose>& additionalPoses);
    void apply(protocol::ContactGloveState_t& state, bool useNN = false);

private:
    size_t convertCalibrationData(const protocol::ContactGloveState_t::HandFingersCalibrationData_t& calibrationData,
        std::vector<GloveModel<double>::GloveValues>& inputs, std::vector<GloveModel<double>::ModelOutputs>& outputs,
        std::vector<GloveModel<double>::ModelOutputComponents>& components);
    GloveModel<double>::GloveModelCalibration modelCalibration;
    NNGloveModel* nn;
};
