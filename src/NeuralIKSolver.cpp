#include "NeuralIKSolver.h"

#include <fstream>
#include <cstdlib>
#include <iostream>

bool NeuralIKSolver::solve(
    double x,
    double y,
    double z,
    JointAngles &result)
{

    // Write target position for Python solver

    std::ofstream file("neural_target.txt");

    if (!file.is_open())
        return false;

    file << x << " "
         << y << " "
         << z;

    file.close();

    // Call Python solver

    int status =
        system("C:/Users/hsiraa/AppData/Local/Programs/Python/Python311/python.exe ../ML/neural_ik_solver.py");

    if (status != 0)
    {
        std::cout << "Neural IK failed\n";
        return false;
    }

    // Read solution

    std::ifstream in("neural_solution.csv");

    if (!in.is_open())
        return false;

    char comma;

    in >> result.t1 >> comma >> result.t2 >> comma >> result.t3;

    return true;
}