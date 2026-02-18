#pragma once
#include <vector>

class Capsule
{
public:
    static double distancePointSegment(
        const std::vector<double> &p,
        const std::vector<double> &a,
        const std::vector<double> &b);

    static std::vector<double> closestPointSegment(
        const std::vector<double> &p,
        const std::vector<double> &a,
        const std::vector<double> &b);
};
