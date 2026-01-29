#pragma once

#include <vector>
#define FOREACH_FINGER(macro, args...) \
    macro(index, args) \
    macro(middle, args) \
    macro(ring, args) \
    macro(pinky, args) \
    macro(thumb, args)
    
#define CAL_VALUE_BOUNDS(value, cal) (((value) - (cal).min) / (float)((cal).max - (cal).min))
#define CAL_BOUNDS_JOINT(value, finger, joint, cal) CAL_VALUE_BOUNDS((value), cal.finger.joint)