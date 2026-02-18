#include "QuinticInterpolator.h"
#include <cmath>

static double quintic(double t)
{
    return 6 * pow(t, 5) - 15 * pow(t, 4) + 10 * pow(t, 3);
}

std::vector<JointAngles>
QuinticInterpolator::interpolate(
    const JointAngles &start,
    const JointAngles &end,
    int steps)
{
    std::vector<JointAngles> traj;

    for (int i = 0; i <= steps; i++)
    {
        double t = (double)i / steps;
        double s = quintic(t);

        JointAngles a;

        a.t1 = start.t1 + s * (end.t1 - start.t1);
        a.t2 = start.t2 + s * (end.t2 - start.t2);
        a.t3 = start.t3 + s * (end.t3 - start.t3);

        traj.push_back(a);
    }

    return traj;
}
