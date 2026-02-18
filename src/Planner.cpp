#include "Planner.h"
#include "CollisionChecker.h"
#include "TrajectoryPlanner.h"

#include <random>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

static std::mt19937 rng(std::random_device{}());

const double STEP = 0.20;
const int MAX_INTERNAL = 1000;
const int INTERPOLATION = 20;

struct Node
{
    JointAngles q;
    int parent;
};

//////////////////////////////////////////////////////////////
// REQUIRED CLASS FUNCTIONS
//////////////////////////////////////////////////////////////

JointAngles Planner::randomConfig()
{
    std::uniform_real_distribution<double> d(-M_PI, M_PI);

    JointAngles q;
    q.t1 = d(rng);
    q.t2 = d(rng);
    q.t3 = d(rng);

    return q;
}

double Planner::distance(
    const JointAngles &a,
    const JointAngles &b)
{
    double d1 = a.t1 - b.t1;
    double d2 = a.t2 - b.t2;
    double d3 = a.t3 - b.t3;

    return sqrt(d1 * d1 + d2 * d2 + d3 * d3);
}

JointAngles Planner::steer(
    const JointAngles &from,
    const JointAngles &to,
    double step)
{
    double d = distance(from, to);

    if (d < step)
        return to;

    double r = step / d;

    JointAngles q;

    q.t1 = from.t1 + (to.t1 - from.t1) * r;
    q.t2 = from.t2 + (to.t2 - from.t2) * r;
    q.t3 = from.t3 + (to.t3 - from.t3) * r;

    return q;
}

bool Planner::isStateValid(
    RobotArm &arm,
    const JointAngles &q,
    const Obstacle &obs)
{
    return !CollisionChecker::checkConfiguration(
        arm, q, obs);
}

//////////////////////////////////////////////////////////////
// INTERNAL HELPERS
//////////////////////////////////////////////////////////////

static bool isPathValid(
    RobotArm &arm,
    const JointAngles &a,
    const JointAngles &b,
    const Obstacle &obs)
{
    for (int i = 0; i <= INTERPOLATION; i++)
    {
        double t = (double)i / INTERPOLATION;

        JointAngles q;

        q.t1 = a.t1 + (b.t1 - a.t1) * t;
        q.t2 = a.t2 + (b.t2 - a.t2) * t;
        q.t3 = a.t3 + (b.t3 - a.t3) * t;

        if (!Planner::isStateValid(arm, q, obs))
            return false;
    }

    return true;
}

static int nearestNode(
    const std::vector<Node> &tree,
    const JointAngles &q)
{
    double best = 1e9;
    int idx = 0;

    for (int i = 0; i < tree.size(); i++)
    {
        double d = Planner::distance(tree[i].q, q);

        if (d < best)
        {
            best = d;
            idx = i;
        }
    }

    return idx;
}

//////////////////////////////////////////////////////////////
// MAIN RRT CONNECT PLANNER
//////////////////////////////////////////////////////////////

std::vector<std::vector<JointAngles>>
Planner::planWithAttempts(
    RobotArm &arm,
    const JointAngles &start,
    const JointAngles &goal,
    const Obstacle &obs,
    int maxSavedAttempts)
{
    std::vector<Node> treeA;
    std::vector<Node> treeB;

    treeA.push_back({start, -1});
    treeB.push_back({goal, -1});

    std::vector<std::vector<JointAngles>> allAttempts;

    bool success = false;
    std::vector<JointAngles> finalPath;

    //////////////////////////////////////////////////////
    // RRT CONNECT LOOP
    //////////////////////////////////////////////////////

    for (int attempt = 0; attempt < MAX_INTERNAL; attempt++)
    {
        JointAngles qrand = randomConfig();

        auto &treeFrom = (attempt % 2 == 0) ? treeA : treeB;
        auto &treeTo = (attempt % 2 == 0) ? treeB : treeA;

        int nearest = nearestNode(treeFrom, qrand);

        JointAngles qnew =
            steer(treeFrom[nearest].q, qrand, STEP);

        if (!isStateValid(arm, qnew, obs))
        {
            std::cout << "Attempt " << attempt + 1 << " collided\n";

            allAttempts.push_back(
                TrajectoryPlanner::generate(start, qnew, 50));

            continue;
        }

        treeFrom.push_back({qnew, nearest});

        //////////////////////////////////////////////////////
        // CONNECT PHASE
        //////////////////////////////////////////////////////

        JointAngles qconnect = qnew;

        while (true)
        {
            int nearestTo = nearestNode(treeTo, qconnect);

            JointAngles qnext =
                steer(treeTo[nearestTo].q, qconnect, STEP);

            if (!isStateValid(arm, qnext, obs))
                break;

            treeTo.push_back({qnext, nearestTo});

            if (distance(qnext, qconnect) < STEP)
            {
                success = true;

                finalPath =
                    TrajectoryPlanner::generate(
                        start, goal, 100);

                allAttempts.push_back(finalPath);

                std::cout << "\nRRT CONNECT SUCCESS\n";

                break;
            }

            qconnect = qnext;
        }

        if (success)
            break;

        allAttempts.push_back(
            TrajectoryPlanner::generate(start, qnew, 50));
    }

    //////////////////////////////////////////////////////
    // FAILURE CASE
    //////////////////////////////////////////////////////

    if (!success)
    {
        std::cout << "\nPLANNER FAILED\n";

        finalPath =
            TrajectoryPlanner::generate(start, goal, 100);

        allAttempts.push_back(finalPath);
    }

    //////////////////////////////////////////////////////
    // SAVE FILES
    //////////////////////////////////////////////////////

    std::vector<std::vector<JointAngles>> saved;

    int total = allAttempts.size();

    if (total <= maxSavedAttempts)
        saved = allAttempts;
    else
    {
        int step = total / maxSavedAttempts;

        for (int i = 0; i < total; i += step)
            saved.push_back(allAttempts[i]);

        saved.push_back(allAttempts.back());
    }

    //////////////////////////////////////////////////////
    // DELETE OLD FILES
    //////////////////////////////////////////////////////

    for (const auto &entry : fs::directory_iterator("."))
    {
        std::string name =
            entry.path().filename().string();

        if (name.find("trajectory_attempt_") != std::string::npos || name.find("trajectory_safe") != std::string::npos)
        {
            fs::remove(entry.path());
        }
    }

    //////////////////////////////////////////////////////
    // SAVE ATTEMPTS
    //////////////////////////////////////////////////////

    for (int i = 0; i < saved.size(); i++)
    {
        std::ofstream f(
            "trajectory_attempt_" + std::to_string(i) + ".txt");

        for (auto &p : saved[i])
            f << p.t1 << " " << p.t2 << " " << p.t3 << "\n";
    }

    //////////////////////////////////////////////////////
    // SAVE SAFE PATH
    //////////////////////////////////////////////////////

    if (success)
    {
        std::ofstream f("trajectory_safe.txt");

        for (auto &p : finalPath)
            f << p.t1 << " " << p.t2 << " " << p.t3 << "\n";
    }

    return saved;
}
