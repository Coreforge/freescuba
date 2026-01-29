#include "cal_poses.h"

const CalibrationPose CalPoses[] = {
    // format:
    // index, middle, ring, pinky, thumb, thumbBase
    // root, tip, splay
    {{{1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, {0., 0., 0.}, {1., 1., 0.}, 1.}, "teatime (pinky)"},
    {{{0., 0., 1.}, {1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, 1.}, "point, index splay to thumb"},
    {{{0., 0., -1.}, {1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, 1.}, "point, index splay away from thumb"},
    {{{0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.}, {0., 1., 0.}, 0.}, "curl only fingertips"},
    {{{1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}, 1.}, "keep finger tips straight, but bend finger bases fully"},
    {{{1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, {0., 0., 0.}, 0.}, "thumbs up"},
    {{{0., 0., 0.}, {1., 1., 0.}, {1., 1., 0.}, {1., 1., 0.}, {0., 0., 0.}, 0.}, "finger guns"},
    {{{0., 0., 0.}, {0., 0., 0.}, {1., 1., 0.}, {1., 1., 0.}, {0., 0., 0.}, 0.}, "finger guns, including middle finger"},
};


const size_t CalPoseCount = sizeof(CalPoses) / sizeof(CalPoses[0]);