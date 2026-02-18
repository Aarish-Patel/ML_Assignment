#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <chrono>

#include "RobotArm.h"
#include "IKSolver.h"
#include "TrajectoryPlanner.h"
#include "CollisionChecker.h"
#include "Obstacle.h"
#include "Planner.h"
#include "NeuralIKSolver.h"

namespace fs = std::filesystem;

//
// =========================
// Compute FK position error
// =========================
//
double computePositionError(
    RobotArm &arm,
    const JointAngles &angles,
    double tx,
    double ty,
    double tz)
{
    double x, y, z;

    arm.forwardKinematics(
        angles.t1,
        angles.t2,
        angles.t3,
        x, y, z);

    return sqrt(
        pow(tx - x, 2) +
        pow(ty - y, 2) +
        pow(tz - z, 2));
}

//
// =========================
// Delete old trajectory files
// =========================
//
void deleteOldTrajectoryFiles()
{
    for (const auto &entry : fs::directory_iterator("."))
    {
        std::string name =
            entry.path().filename().string();

        if (name.find("trajectory_") != std::string::npos &&
            entry.path().extension() == ".txt")
        {
            fs::remove(entry.path());
        }
    }
}

//
// =========================
// MAIN
// =========================
//
int main()
{
    deleteOldTrajectoryFiles();

    //
    // =========================
    // Robot setup
    // =========================
    //
    RobotArm arm(0.3, 0.25, 0.15);
    IKSolver ik(0.3, 0.25, 0.15);

    //
    // =========================
    // Obstacle
    // =========================
    //
    Obstacle obs;
    obs.x = 0.15;
    obs.y = -0.05;
    obs.z = 0.37;
    obs.radius = 0.08;

    //
    // =========================
    // Target
    // =========================
    //
    double target_x = 0.28;
    double target_y = 0.2;
    double target_z = 0.42;

    //
    // Save scene for visualizer
    //
    std::ofstream scene("scene.txt");

    scene << obs.x << " "
          << obs.y << " "
          << obs.z << " "
          << obs.radius << "\n";

    scene << target_x << " "
          << target_y << " "
          << target_z << "\n";

    scene.close();

    //
    // =========================
    // Analytical IK timing
    // =========================
    //
    auto start_analytic =
        std::chrono::high_resolution_clock::now();

    auto solutions =
        ik.solveAll(
            target_x,
            target_y,
            target_z);

    auto end_analytic =
        std::chrono::high_resolution_clock::now();

    double analytic_time =
        std::chrono::duration<double, std::micro>(
            end_analytic - start_analytic)
            .count();

    if (solutions.empty())
    {
        std::cout << "Analytical IK failed\n";
        return 0;
    }

    JointAngles analytic = solutions[0];

    std::cout << "\nAnalytical IK solution:\n";
    std::cout
        << analytic.t1 << " "
        << analytic.t2 << " "
        << analytic.t3 << "\n";

    //
    // =========================
    // Neural IK timing
    // =========================
    //
    JointAngles neural;

    auto start_neural =
        std::chrono::high_resolution_clock::now();

    if (!NeuralIKSolver::solve(
            target_x,
            target_y,
            target_z,
            neural))
    {
        std::cout << "Neural IK failed\n";
        return 0;
    }

    auto end_neural =
        std::chrono::high_resolution_clock::now();

    double neural_time =
        std::chrono::duration<double, std::milli>(
            end_neural - start_neural)
            .count();

    std::cout << "\nNeural IK solution:\n";
    std::cout
        << neural.t1 << " "
        << neural.t2 << " "
        << neural.t3 << "\n";

    //
    // =========================
    // Accuracy comparison
    // =========================
    //
    double analytic_error =
        computePositionError(
            arm,
            analytic,
            target_x,
            target_y,
            target_z);

    double neural_error =
        computePositionError(
            arm,
            neural,
            target_x,
            target_y,
            target_z);

    double joint_error =
        sqrt(
            pow(neural.t1 - analytic.t1, 2) +
            pow(neural.t2 - analytic.t2, 2) +
            pow(neural.t3 - analytic.t3, 2));

    //
    // =========================
    // Print comparison
    // =========================
    //
    std::cout << "\n========== PERFORMANCE COMPARISON ==========\n";

    std::cout << "\nAccuracy:\n";

    std::cout
        << "Analytical position error: "
        << analytic_error
        << " meters\n";

    std::cout
        << "Neural position error: "
        << neural_error
        << " meters\n";

    std::cout
        << "Joint angle difference: "
        << joint_error
        << " radians\n";

    std::cout << "\nSpeed:\n";

    std::cout
        << "Analytical IK time: "
        << analytic_time
        << " microseconds\n";

    std::cout
        << "Neural IK time: "
        << neural_time
        << " milliseconds\n";

    //
    // =========================
    // Trajectory planning
    // =========================
    //
    JointAngles start = {0, 0, 0};

    auto analytic_attempts =
        Planner::planWithAttempts(
            arm,
            start,
            analytic,
            obs,
            10);

    auto neural_attempts =
        Planner::planWithAttempts(
            arm,
            start,
            neural,
            obs,
            10);

    if (analytic_attempts.empty())
    {
        std::cout << "Analytical planner failed\n";
        return 0;
    }

    if (neural_attempts.empty())
    {
        std::cout << "Neural planner failed\n";
        return 0;
    }

    auto safe_analytic =
        analytic_attempts.back();

    auto safe_neural =
        neural_attempts.back();

    //
    // =========================
    // Save trajectories
    // =========================
    //
    std::ofstream analyticFile(
        "trajectory_safe_analytic.txt");

    for (auto &p : safe_analytic)
    {
        analyticFile
            << p.t1 << " "
            << p.t2 << " "
            << p.t3 << "\n";
    }

    analyticFile.close();

    std::ofstream neuralFile(
        "trajectory_safe_neural.txt");

    for (auto &p : safe_neural)
    {
        neuralFile
            << p.t1 << " "
            << p.t2 << " "
            << p.t3 << "\n";
    }

    neuralFile.close();

    std::cout << "\nTrajectory files written successfully\n";

    return 0;
}
