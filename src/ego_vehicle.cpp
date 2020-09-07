#include "ego_vehicle.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"

#include <cmath>

void EgoVehicle::UpdateExistingTrajectory(const TrajectoryPts& prev_trajectory)
{
    trajectory_ = prev_trajectory;
}

// void EgoVehicle::GenerateAnchorPts()
// {

// }


void EgoVehicle::UpdatePosition(const Vehicle& localization_data, const TrajectoryPts& prev_trajectory)
{
    x_ = localization_data.x;
    y_ = localization_data.y;
    s_ = localization_data.s;
    d_ = localization_data.d;
    yaw_ = localization_data.yaw;
    velocity_ = localization_data.velocity;
    end_path_s_ = localization_data.end_path_s;
    end_path_d_ = localization_data.end_path_d;

    UpdateExistingTrajectory(prev_trajectory);
}


void EgoVehicle::ExecuteKeepLane(const std::vector<Vehicle>& predictions)
{

}

void EgoVehicle::PlanMission(const std::vector<Vehicle>& predictions)
{
    switch (mission_)
    {
    case Mission::kKeepLane:
        ExecuteKeepLane(predictions);
        break;

    default:
        break;
    }
}


TrajectoryPts EgoVehicle::GenerateTrajectory(const WorldMap& map)
{
    TrajectoryPts new_trajectory;

    const auto prev_size = trajectory_.x_pts.size();

    double ref_x = x_;
    double ref_y = y_;
    double ref_yaw = deg2rad(yaw_);

    if (prev_size < 2)
    {
        // Generate new EgoVehicle
        const double prev_car_x = x_ - std::cos(yaw_);
        const double prev_car_y = y_ - std::sin(yaw_);

        new_trajectory.x_pts.push_back(prev_car_x);
        new_trajectory.x_pts.push_back(ref_x);

        new_trajectory.y_pts.push_back(prev_car_y);
        new_trajectory.y_pts.push_back(ref_y);
    }
    else
    {
        // Concatanate new EgoVehicle points to the previous one
        ref_x = trajectory_.x_pts[prev_size - 1];
        ref_y = trajectory_.y_pts[prev_size - 1];

        const double prev_ref_x = trajectory_.x_pts[prev_size - 2];
        const double prev_ref_y = trajectory_.y_pts[prev_size - 2];

        double ref_yaw = std::atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

        new_trajectory.x_pts.push_back(prev_ref_x);
        new_trajectory.x_pts.push_back(ref_x);

        new_trajectory.y_pts.push_back(prev_ref_y);
        new_trajectory.y_pts.push_back(ref_y);
    }

    // Generate 3 points that represent main (anchor) points where spline has to connect
    std::vector<double> next_wp0{getXY(s_ + 30, (kLaneCenter + kLaneWidth * current_lane_), map.waypoints_s, map.waypoints_x, map.waypoints_y)};
    std::vector<double> next_wp1{getXY(s_ + 60, (kLaneCenter + kLaneWidth * current_lane_), map.waypoints_s, map.waypoints_x, map.waypoints_y)};
    std::vector<double> next_wp2{getXY(s_ + 90, (kLaneCenter + kLaneWidth * current_lane_), map.waypoints_s, map.waypoints_x, map.waypoints_y)};

    new_trajectory.x_pts.push_back(next_wp0[0]);
    new_trajectory.x_pts.push_back(next_wp1[0]);
    new_trajectory.x_pts.push_back(next_wp2[0]);

    new_trajectory.y_pts.push_back(next_wp0[1]);
    new_trajectory.y_pts.push_back(next_wp1[1]);
    new_trajectory.y_pts.push_back(next_wp2[1]);

    for (int i = 0; i < new_trajectory.x_pts.size(); ++i)
    {
        double shift_x = new_trajectory.x_pts[i] - ref_x;
        double shift_y = new_trajectory.y_pts[i] - ref_y;

        new_trajectory.x_pts[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 -ref_yaw));
        new_trajectory.y_pts[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 -ref_yaw));
    }


    for (int i = 0; i < prev_size; ++i)
    {
        new_trajectory.x_pts.push_back(trajectory_.x_pts[i]);
        new_trajectory.y_pts.push_back(trajectory_.y_pts[i]);
    }

    // Generate new points
    tk::spline s;
    s.set_points(new_trajectory.x_pts, new_trajectory.y_pts);

    // How far we predict along the road
    double target_x = kPredictionTarget;
    double target_y = s(target_x);  // Get y coordinate from spline

    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

    double x_add_on = 0;

    for(int i = 1; i <= 50 - prev_size; ++i)
    {
        double N = (target_dist / (.02 * reference_velocity_ / 2.24));
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        new_trajectory.x_pts.push_back(x_point);
        new_trajectory.y_pts.push_back(y_point);
    }

    trajectory_ = new_trajectory;

    return new_trajectory;
}