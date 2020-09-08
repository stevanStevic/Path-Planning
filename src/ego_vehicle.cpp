#include "ego_vehicle.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "road.h"
#include "spline.h"

#include <cmath>
#include <algorithm>
#include <iostream>

void EgoVehicle::UpdateExistingTrajectory(const TrajectoryPts& prev_trajectory)
{
    trajectory_ = prev_trajectory;
}

void EgoVehicle::UpdatePosition(const Vehicle& localization_data, const TrajectoryPts& prev_trajectory)
{
    int prev_size = prev_trajectory.x_pts.size();

    // Sensor fusion
    if (prev_size > 2)
    {
        s_ = localization_data.end_path_s;
    }
    else
    {
        s_ = localization_data.s;
    }


    x_ = localization_data.x;
    y_ = localization_data.y;
    d_ = localization_data.d;
    yaw_ = localization_data.yaw;
    velocity_ = localization_data.velocity;
    end_path_s_ = localization_data.end_path_s;
    end_path_d_ = localization_data.end_path_d;

    UpdateExistingTrajectory(prev_trajectory);
}

std::vector<Mission> EgoVehicle::GetPossibleMissions() const
{
    std::vector<Mission> possible_missions;

    possible_missions.push_back(Mission::kKeepLane);

    if (Mission::kKeepLane == mission_)
    {
        if (road_.IsLaneChangePossible(current_lane_, Mission::kChangeLaneLeft))
        {
            possible_missions.push_back(Mission::kChangeLaneLeft);
        }

        if(road_.IsLaneChangePossible(current_lane_, Mission::kChangeLaneRight))
        {
            possible_missions.push_back(Mission::kChangeLaneRight);
        }
    }

    return possible_missions;
}

void EgoVehicle::ChooseNextMission(const Road& road)
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

        if (road_.GetNearestVehicle(vehicle_ahead, s_, new_lane))
        {
            // The slower the lane, higher the cost
            cost = 15000 * sigmoid(vehicle_ahead.velocity / kSpeedLimit, 1.0);

            // Punish beign too close
            cost += 35000 * sigmoid(1.0, std::fabs(vehicle_ahead.s - s_));
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

void EgoVehicle::ExecuteKeepLane(const Road& road)
{
    bool too_close{false};
    Vehicle vehicle_ahead;

    if (road_.GetVehicleAhead(vehicle_ahead, s_, current_lane_))
    {
        const double gap = (vehicle_ahead.s - s_);
        if (gap < kPrefferedFrontGap)
        {
            ChooseNextMission(road);

            if (Mission::kKeepLane == mission_)
            {
                // If mission is still keep lane ,then slowdown
                too_close = true;
            }
        }
    }

    if(too_close && (reference_velocity_ > vehicle_ahead.velocity))
    {
        reference_velocity_ -= kMaxAcceleration;
    }
    else if(reference_velocity_ < kSpeedLimit)
    {
        reference_velocity_ += kMaxAcceleration;
    }
}

void EgoVehicle::ExecuteLaneChange(const Road& road)
{
    if (road.IsLaneTransitionCompleted(d_, current_lane_));
    {
        mission_ = Mission::kKeepLane;
    }
}

void EgoVehicle::PlanMission(const Road& road)
{
    switch (mission_)
    {
    case Mission::kKeepLane:
    {
        ExecuteKeepLane(road);
        break;
    }
    case Mission::kChangeLaneLeft:
    case Mission::kChangeLaneRight:
    {
        ExecuteLaneChange(road);
        break;
    }
    default:
        break;
    }
}
