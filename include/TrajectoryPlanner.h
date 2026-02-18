#pragma once

#include <vector>
#include "JointAngles.h"

class TrajectoryPlanner
{
public:
    static std::vector<JointAngles>
    generate(
        const JointAngles &start,
        const JointAngles &goal,
        int steps);

private:
    static double quintic(double t);
};
