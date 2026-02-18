#include "IKSolver.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

IKSolver::IKSolver(double l1, double l2, double l3)
{
    L1 = l1;
    L2 = l2;
    L3 = l3;
}

bool IKSolver::solve(
    double x,
    double y,
    double z,
    double &t1,
    double &t2,
    double &t3)
{

    // Base rotation
    t1 = atan2(y, x);

    // Horizontal distance
    double r = sqrt(x * x + y * y);

    // Vertical offset from shoulder
    double z_offset = z - L1;

    // Distance from shoulder to target
    double d =
        sqrt(r * r + z_offset * z_offset);

    // Check reachability
    if (d > (L2 + L3) || d < fabs(L2 - L3))
        return false;

    // Elbow angle using cosine law
    double cos_t3 =
        (d * d - L2 * L2 - L3 * L3) /
        (2 * L2 * L3);

    cos_t3 = std::max(-1.0, std::min(1.0, cos_t3));

    t3 = acos(cos_t3);

    // Shoulder angle
    double alpha = atan2(z_offset, r);

    double beta =
        atan2(
            L3 * sin(t3),
            L2 + L3 * cos(t3));

    t2 = alpha - beta;

    if (t1 < -M_PI || t1 > M_PI)
        return false;
    if (t2 < -M_PI || t2 > M_PI)
        return false;
    if (t3 < -M_PI || t3 > M_PI)
        return false;

    return true;
}

std::vector<JointAngles>
IKSolver::solveAll(double x, double y, double z)
{

    std::vector<JointAngles> solutions;

    double t1 = atan2(y, x);

    double r = sqrt(x * x + y * y);
    double z_offset = z - L1;

    double d = sqrt(r * r + z_offset * z_offset);

    double cos_t3 =
        (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3);

    if (cos_t3 < -1 || cos_t3 > 1)
        return solutions;

    // Elbow down
    double t3a = acos(cos_t3);

    double alpha = atan2(z_offset, r);

    double beta =
        atan2(L3 * sin(t3a),
              L2 + L3 * cos(t3a));

    double t2a = alpha - beta;

    solutions.push_back(
        {t1, t2a, t3a});

    // Elbow up
    double t3b = -acos(cos_t3);

    beta =
        atan2(L3 * sin(t3b),
              L2 + L3 * cos(t3b));

    double t2b = alpha - beta;

    solutions.push_back(
        {t1, t2b, t3b});

    return solutions;
}
