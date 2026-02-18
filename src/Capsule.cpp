#include "Capsule.h"
#include <cmath>

double Capsule::distancePointSegment(
    const std::vector<double> &p,
    const std::vector<double> &a,
    const std::vector<double> &b)
{
    auto c =
        closestPointSegment(p, a, b);

    double dx = p[0] - c[0];
    double dy = p[1] - c[1];
    double dz = p[2] - c[2];

    return sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<double>
Capsule::closestPointSegment(
    const std::vector<double> &p,
    const std::vector<double> &a,
    const std::vector<double> &b)
{
    double vx = b[0] - a[0];
    double vy = b[1] - a[1];
    double vz = b[2] - a[2];

    double wx = p[0] - a[0];
    double wy = p[1] - a[1];
    double wz = p[2] - a[2];

    double c1 = vx * wx + vy * wy + vz * wz;
    double c2 = vx * vx + vy * vy + vz * vz;

    double t = c1 / c2;

    if (t < 0)
        t = 0;
    if (t > 1)
        t = 1;

    return {
        a[0] + t * vx,
        a[1] + t * vy,
        a[2] + t * vz};
}
