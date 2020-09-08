#include "spline_trajectory.h"

#include "ego_vehicle.h"
#include "helpers.h"
#include "road.h"
#include "spline.h"


TrajectoryPts SplineTrajectory::GenerateAnchorWaypoints(const EgoVehicle& ego_vehicle,
                                                        const Road& road,
                                                        const WorldMap& map,
                                                        double& ref_x,
                                                        double& ref_y,
                                                        double& ref_yaw)
{
    TrajectoryPts new_trajectory;

    const auto prev_size = ego_vehicle.trajectory_.x_pts.size();

    if (prev_size < 2)
    {
        // Generate new EgoVehicle
        const double prev_car_x = ref_x - std::cos(ref_yaw);
        const double prev_car_y = ref_y - std::sin(ref_yaw);

        new_trajectory.x_pts.push_back(prev_car_x);
        new_trajectory.x_pts.push_back(ref_x);

        new_trajectory.y_pts.push_back(prev_car_y);
        new_trajectory.y_pts.push_back(ref_y);
    }
    else
    {
        // Concatanate new EgoVehicle points to the previous one
        ref_x = ego_vehicle.trajectory_.x_pts[prev_size - 1];
        ref_y = ego_vehicle.trajectory_.y_pts[prev_size - 1];

        const double prev_ref_x = ego_vehicle.trajectory_.x_pts[prev_size - 2];
        const double prev_ref_y = ego_vehicle.trajectory_.y_pts[prev_size - 2];

        double ref_yaw = std::atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

        new_trajectory.x_pts.push_back(prev_ref_x);
        new_trajectory.x_pts.push_back(ref_x);

        new_trajectory.y_pts.push_back(prev_ref_y);
        new_trajectory.y_pts.push_back(ref_y);
    }

    // Generate 3 points that represent main (anchor) points where spline has to connect

    for (auto i = 1; i <= 3; ++i)
    {
        const double anchor_s{ego_vehicle.pos_.s + kTrajectoryHorizon * i};
        const double anchor_d{road.kLaneCenterOffset + (road.kLaneWidth * ego_vehicle.current_lane_)};

        std::vector<double> next_wp{getXY(anchor_s, anchor_d, map.waypoints_s, map.waypoints_x, map.waypoints_y)};

        new_trajectory.x_pts.push_back(next_wp[0]);
        new_trajectory.y_pts.push_back(next_wp[1]);
    }

    // Transform from world coordinate to local (vehicle) coordinates for easier math
    for (int i = 0; i < new_trajectory.x_pts.size(); ++i)
    {
        double shift_x = new_trajectory.x_pts[i] - ref_x;
        double shift_y = new_trajectory.y_pts[i] - ref_y;

        new_trajectory.x_pts[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 -ref_yaw));
        new_trajectory.y_pts[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 -ref_yaw));
    }

    return new_trajectory;
}


void SplineTrajectory::SplineConnectWaypoints(TrajectoryPts& new_trajectory,
                                              const EgoVehicle& ego_vehicle,
                                              const Road& road,
                                              double& ref_x,
                                              double& ref_y,
                                              double& ref_yaw)
{
    const auto prev_size = ego_vehicle.trajectory_.x_pts.size();

    // Generate new points
    tk::spline s;
    s.set_points(new_trajectory.x_pts, new_trajectory.y_pts);

    for (int i = 0; i < prev_size; ++i)
    {
        new_trajectory.x_pts.push_back(ego_vehicle.trajectory_.x_pts[i]);
        new_trajectory.y_pts.push_back(ego_vehicle.trajectory_.y_pts[i]);
    }

    // How far we predict along the road
    double target_x = kTrajectoryHorizon;
    double target_y = s(target_x);  // Get y coordinate from spline

    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

    double x_add_on = 0;

    for(int i = 1; i <= 50 - prev_size; ++i)
    {
        const double velocity_in_mph = ego_vehicle.reference_velocity_ / 2.24; // converstion from kmph -> mph

        double N = (target_dist / (road.kSimulationTimeStep * velocity_in_mph)); // Find number of points along spline
        double x_point = x_add_on + target_x/N; // Project them on the x axis
        double y_point = s(x_point);            // Get cooresponding y cooridnate from spline

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Convert back to world map
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        new_trajectory.x_pts.push_back(x_point);
        new_trajectory.y_pts.push_back(y_point);
    }
}

const TrajectoryPts SplineTrajectory::GenerateTrajectory(const EgoVehicle& ego_vehicle,
                                                         const Road& road,
                                                         const WorldMap& map)
{
    double ref_x = ego_vehicle.pos_.x;
    double ref_y = ego_vehicle.pos_.y;
    double ref_yaw = deg2rad(ego_vehicle.pos_.yaw);

    TrajectoryPts new_trajectory = GenerateAnchorWaypoints(ego_vehicle, road, map, ref_x, ref_y, ref_yaw);

    SplineConnectWaypoints(new_trajectory, ego_vehicle, road, ref_x, ref_y, ref_yaw);

    return new_trajectory;
}