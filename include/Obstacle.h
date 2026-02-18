#pragma once

enum class ObstacleType
{
    SPHERE,
    CUBE,
    CUBOID
};

struct Obstacle
{
    ObstacleType type;

    double x, y, z;

    double radius;

    double sx, sy, sz;
};
