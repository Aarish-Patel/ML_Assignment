#include "RobotArm.h"
#include <cmath>

RobotArm::RobotArm(double l1, double l2, double l3)
{
    L1 = l1;
    L2 = l2;
    L3 = l3;
}

void RobotArm::forwardKinematics(
    double t1, double t2, double t3,
    double &x, double &y, double &z)
{
    double x2 = L2 * cos(t2) * cos(t1);
    double y2 = L2 * cos(t2) * sin(t1);
    double z2 = L1 + L2 * sin(t2);

    x = x2 + L3 * cos(t2 + t3) * cos(t1);
    y = y2 + L3 * cos(t2 + t3) * sin(t1);
    z = z2 + L3 * sin(t2 + t3);
}

std::vector<std::vector<double>>
RobotArm::getJointPositions(const JointAngles &a)
{
    std::vector<std::vector<double>> joints(4);

    joints[0] = {0, 0, 0};

    joints[1] = {0, 0, L1};

    joints[2] =
        {
            L2 * cos(a.t2) * cos(a.t1),
            L2 * cos(a.t2) * sin(a.t1),
            L1 + L2 * sin(a.t2)};

    joints[3] =
        {
            joints[2][0] + L3 * cos(a.t2 + a.t3) * cos(a.t1),
            joints[2][1] + L3 * cos(a.t2 + a.t3) * sin(a.t1),
            joints[2][2] + L3 * sin(a.t2 + a.t3)};

    return joints;
}

double RobotArm::getLinkRadius() const
{
    return 0.02;
}
