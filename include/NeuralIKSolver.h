#ifndef NEURAL_IK_SOLVER_H
#define NEURAL_IK_SOLVER_H

#include "JointAngles.h"

class NeuralIKSolver
{
public:
    static bool solve(
        double x,
        double y,
        double z,
        JointAngles &result);
};

#endif
