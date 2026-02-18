#include "CollisionChecker.h"
#include "Capsule.h"
#include <limits>

bool CollisionChecker::checkConfiguration(
    RobotArm &arm,
    const JointAngles &angles,
    const Obstacle &obs)
{
    return checkConfigurationDetailed(
               arm,
               angles,
               obs)
        .collided;
}

CollisionInfo CollisionChecker::checkConfigurationDetailed(
    RobotArm &arm,
    const JointAngles &angles,
    const Obstacle &obs)
{
    auto joints = arm.getJointPositions(angles);

    double linkRadius = arm.getLinkRadius();

    std::vector<double> center =
        {obs.x, obs.y, obs.z};

    CollisionInfo info;
    info.collided = false;
    info.linkIndex = -1;

    double bestDist =
        std::numeric_limits<double>::max();

    for (int i = 0; i < 3; i++)
    {
        double dist =
            Capsule::distancePointSegment(
                center,
                joints[i],
                joints[i + 1]);

        if (dist < bestDist)
        {
            bestDist = dist;
            info.linkIndex = i;
            info.point =
                Capsule::closestPointSegment(
                    center,
                    joints[i],
                    joints[i + 1]);
        }

        if (dist < linkRadius + obs.radius)
        {
            info.collided = true;
            info.linkIndex = i;
            info.point =
                Capsule::closestPointSegment(
                    center,
                    joints[i],
                    joints[i + 1]);
            return info;
        }
    }

    return info;
}
