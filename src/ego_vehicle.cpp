#include "ego_vehicle.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"

#include <cmath>
#include <algorithm>
#include <iostream>

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

bool EgoVehicle::IsLaneChangePossible(const Mission mission) const
{
    const int8_t new_lane = current_lane_ + static_cast<int8_t>(mission);

    return (new_lane >= 0) && (new_lane < kNumOfLanes);
}

std::vector<Mission> EgoVehicle::GetPossibleMissions()
{
    std::vector<Mission> possible_missions;

    possible_missions.push_back(Mission::kKeepLane);

    if (Mission::kKeepLane == mission_)
    {
        if (IsLaneChangePossible(Mission::kChangeLaneLeft))
        {
            possible_missions.push_back(Mission::kChangeLaneLeft);
        }

        if(IsLaneChangePossible(Mission::kChangeLaneRight))
        {
            possible_missions.push_back(Mission::kChangeLaneRight);
        }
    }

    return possible_missions;
}

double Sigmoid(const double& x, const double& y)
{

    return 1 - exp((-x) / y);
}

// double EgoVehicle::cost(const std::vector<Vehicle>& predictions)

void EgoVehicle::ChooseNextMission(const std::vector<Vehicle>& predictions)
{
    // Check cost of staying in the same lane
    const auto possible_missions = GetPossibleMissions();
    std::vector<double> costs;

    for (const Mission mission : possible_missions)
    {
        Vehicle vehicle_ahead;
        double cost = 0;

        // For every state calucalte cost
        auto new_lane = static_cast<uint8_t>(current_lane_ + static_cast<int8_t>(mission));
        std::cout << "New lane: " << (int)new_lane << std::endl;
        if (GetVehicleAhead(predictions, vehicle_ahead, new_lane))
        {
            // The slower the lane, higher the cost
            cost = 15000 * Sigmoid(vehicle_ahead.velocity / kSpeedLimit, 1.0);

            // Punish beign too close
            cost += 35000 * Sigmoid(1.0, vehicle_ahead.s - s_);
        }

        costs.push_back(cost);
    }



    int min_index = -1;
    double lowest_cost = std::numeric_limits<double>::max();

    for (auto i = 0; i < costs.size(); ++i)
    {
        if (lowest_cost > costs[i])
        {
            lowest_cost = costs[i];
            min_index = i;
        }
    }

    mission_ == possible_missions[min_index];

    current_lane_ += static_cast<int8_t>(possible_missions[min_index]);
}

void EgoVehicle::ExecuteKeepLane(const std::vector<Vehicle>& predictions)
{
    Vehicle vehicle_ahead;
    ChooseNextMission(predictions);

    // if (GetVehicleAhead(predictions, vehicle_ahead, current_lane_))
    // {

    // }
}

bool EgoVehicle::IsLaneTransitionCompleted() const
{
    const double reference_lane_center = kLaneCenterOffset + current_lane_ * kLaneWidth;

    const double diff_to_lane_center = std::fabs(reference_lane_center - d_);

    return diff_to_lane_center < 0.2;
}

void EgoVehicle::ExecuteLaneChange()
{
    if (IsLaneTransitionCompleted())
    {
        mission_ = Mission::kKeepLane;
    }
}

void EgoVehicle::PlanMission(const std::vector<Vehicle>& predictions)
{
    switch (mission_)
    {
    case Mission::kKeepLane:
    {
        ExecuteKeepLane(predictions);
        break;
    }
    case Mission::kChangeLaneLeft:
    case Mission::kChangeLaneRight:
    {
        ExecuteLaneChange();
        break;
    }
    default:
        break;
    }
}

bool EgoVehicle::GetVehicleAhead(const std::vector<Vehicle>& predictions,
                                Vehicle& vehicle_ahead,
                                uint8_t ego_lane)
{
    bool found_vehicle{false};
    double closest_vehicle_distance = std::numeric_limits<double>::max();

    for (const auto& vehicle : predictions)
    {
        const float d = vehicle.d;
        // Check if detected vehicle is in lane of ego vehicle
        if ((d > ego_lane * kLaneWidth) && (d < (ego_lane * kLaneWidth + kLaneWidth)))
        {
            // Check if it is closer
            // double distance_diff = std::fabs(vehicle.s) - s_;
            if (vehicle.s > s_ && vehicle.s < closest_vehicle_distance)
            {
                closest_vehicle_distance = vehicle.s;
                vehicle_ahead = vehicle;
                found_vehicle = true;
            }
        }
    }

    return found_vehicle;
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
    std::vector<double> next_wp0{getXY(s_ + 30, (kLaneCenterOffset + kLaneWidth * current_lane_), map.waypoints_s, map.waypoints_x, map.waypoints_y)};
    std::vector<double> next_wp1{getXY(s_ + 60, (kLaneCenterOffset + kLaneWidth * current_lane_), map.waypoints_s, map.waypoints_x, map.waypoints_y)};
    std::vector<double> next_wp2{getXY(s_ + 90, (kLaneCenterOffset + kLaneWidth * current_lane_), map.waypoints_s, map.waypoints_x, map.waypoints_y)};

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

    // Generate new points
    tk::spline s;
    s.set_points(new_trajectory.x_pts, new_trajectory.y_pts);

    for (int i = 0; i < prev_size; ++i)
    {
        new_trajectory.x_pts.push_back(trajectory_.x_pts[i]);
        new_trajectory.y_pts.push_back(trajectory_.y_pts[i]);
    }

    // How far we predict along the road
    double target_x = kPredictionHorizon;
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