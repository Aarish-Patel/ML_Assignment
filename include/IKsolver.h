#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <vector>
#include "JointAngles.h"

class IKSolver
{
public:
    IKSolver(double L1, double L2, double L3);

    bool solve(
        double x, double y, double z,
        double &t1, double &t2, double &t3);

    std::vector<JointAngles>
    solveAll(double x, double y, double z);

private:
    double L1, L2, L3;
};

#endif
