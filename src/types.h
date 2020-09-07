#ifndef TYPES_H
#define TYPES_H

#include <vector>

struct TrajectoryPts
{
    std::vector<double> x_pts;
    std::vector<double> y_pts;
};

struct Vehicle
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double velocity;
    double end_path_s;
    double end_path_d;
};

struct WorldMap
{
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;
};

enum class Mission
{
    kKeepLane,
    kChangeLaneLeft,
    kChaneLaneRight
};

#endif
