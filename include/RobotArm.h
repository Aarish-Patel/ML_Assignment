#pragma once

#include "JointAngles.h"
#include <vector>

class RobotArm
{
private:
    double L1;
    double L2;
    double L3;

public:
    RobotArm(double l1, double l2, double l3);

    void forwardKinematics(
        double t1,
        double t2,
        double t3,
        double &x,
        double &y,
        double &z);

    std::vector<std::vector<double>>
    getJointPositions(const JointAngles &angles);

    double getLinkRadius() const;

    double getL1() const { return L1; }
    double getL2() const { return L2; }
    double getL3() const { return L3; }
};
