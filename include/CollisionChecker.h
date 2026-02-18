#pragma once

#include "RobotArm.h"
#include "JointAngles.h"
#include "Obstacle.h"
#include <vector>

struct CollisionInfo
{
    bool collided;
    int linkIndex;
    std::vector<double> point;
};

class CollisionChecker
{
public:
    static bool checkConfiguration(
        RobotArm &arm,
        const JointAngles &angles,
        const Obstacle &obs);

    static CollisionInfo checkConfigurationDetailed(
        RobotArm &arm,
        const JointAngles &angles,
        const Obstacle &obs);
};
