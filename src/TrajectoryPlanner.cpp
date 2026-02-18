#include "TrajectoryPlanner.h"

double TrajectoryPlanner::quintic(double t)
{
    return 10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t;
}

std::vector<JointAngles>
TrajectoryPlanner::generate(
    const JointAngles &start,
    const JointAngles &goal,
    int steps)
{
    std::vector<JointAngles> traj;

    for (int i = 0; i <= steps; i++)
    {
        double t = (double)i / steps;
        double s = quintic(t);

        traj.push_back(
            {start.t1 + (goal.t1 - start.t1) * s,
             start.t2 + (goal.t2 - start.t2) * s,
             start.t3 + (goal.t3 - start.t3) * s});
    }

    return traj;
}
