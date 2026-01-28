#pragma once
#include "glove_model.hpp"

enum CalibrationPoseNames{
    CALPOSE_TEATIME,

    CALPOSE_COUNT
};

struct CalibrationPose{
    GloveModel<double>::ModelOutputs pose;
    const char* text;
};

struct RecordedCalibrationPose{
    GloveModel<double>::ModelOutputs pose;
    GloveModel<double>::GloveValues sensors;
};

extern const CalibrationPose CalPoses[];
extern const size_t CalPoseCount;