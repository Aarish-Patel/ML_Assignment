#pragma once

#include "RobotArm.h"
#include "Obstacle.h"
#include "JointAngles.h"
#include <vector>

class RRTConnect
{
public:
    static bool plan(
        RobotArm &arm,
        const JointAngles &start,
        const JointAngles &goal,
        const Obstacle &obs,
        std::vector<JointAngles> &path);
};
