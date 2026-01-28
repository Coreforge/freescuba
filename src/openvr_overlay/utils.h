#pragma once

#include <vector>
#define FOREACH_FINGER(macro, args...) \
    macro(index, args) \
    macro(middle, args) \
    macro(ring, args) \
    macro(pinky, args) \
    macro(thumb, args)
    
