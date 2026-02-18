#pragma once

#include "JointAngles.h"
#include <vector>

class QuinticInterpolator
{
public:
    static std::vector<JointAngles>
    interpolate(
        const JointAngles &start,
        const JointAngles &end,
        int steps);
};
