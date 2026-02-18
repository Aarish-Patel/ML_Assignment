#pragma once

#include "JointAngles.h"
#include "RobotArm.h"
#include "Obstacle.h"
#include <vector>

class Planner
{
public:
    // MAIN FUNCTION
    static std::vector<std::vector<JointAngles>>
    planWithAttempts(
        RobotArm &arm,
        const JointAngles &start,
        const JointAngles &goal,
        const Obstacle &obs,
        int maxSavedAttempts);

    //////////////////////////////////////////////////////
    // MAKE THESE PUBLIC (FIXES YOUR ERROR)
    //////////////////////////////////////////////////////

    static JointAngles randomConfig();

    static double distance(
        const JointAngles &a,
        const JointAngles &b);

    static JointAngles steer(
        const JointAngles &from,
        const JointAngles &to,
        double step);

    static bool isStateValid(
        RobotArm &arm,
        const JointAngles &q,
        const Obstacle &obs);
};
