#include "ego_vehicle.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "road.h"
#include "spline.h"

#include <cmath>
#include <algorithm>
#include <iostream>

void EgoVehicle::UpdateExistingTrajectory(const TrajectoryPts& trajectory)
{
    trajectory_ = trajectory;
}

void EgoVehicle::UpdatePosition(const Vehicle& localization_data, const TrajectoryPts& prev_trajectory)
{
    pos_ = localization_data;

    int prev_size = prev_trajectory.x_pts.size();

    if (prev_size > 2)
    {
        // Take trajectory end
        pos_.s = localization_data.end_path_s;
    }

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

const double EgoVehicle::CalculateMissionCost(const Mission mission) const
{
    Vehicle vehicle_ahead;
    double cost{0};

    // End lane for given mission
    const auto new_lane = static_cast<uint8_t>(current_lane_ + static_cast<int8_t>(mission));

    if (road_.GetNearestVehicle(vehicle_ahead, pos_.s, new_lane))
    {
        // The slower the lane, higher the cost
        cost = 15000 * sigmoid(vehicle_ahead.velocity / road_.kSpeedLimit, 1.0);

        // Punish beign too close
        cost += 35000 * sigmoid(1.0, std::fabs(vehicle_ahead.s - pos_.s));
    }

    return cost;
}

const int EgoVehicle::GetLowestCostIndx(const std::vector<double>& costs) const
{
    int lowest_cost_indx = -1;
    double lowest_cost = std::numeric_limits<double>::max();

    for (auto i = 0; i < costs.size(); ++i)
    {
        if (lowest_cost > costs[i])
        {
            lowest_cost = costs[i];
            lowest_cost_indx = i;
        }
    }

    return lowest_cost_indx;
}

void EgoVehicle::ChooseNextMission()
{
    // Check cost of staying in the same lane
    const auto possible_missions = GetPossibleMissions();
    std::vector<double> costs;

    for (const Mission mission : possible_missions)
    {
        costs.push_back(CalculateMissionCost(mission));
    }

    const int lowest_cost_indx{GetLowestCostIndx(costs)};

    mission_ = possible_missions[lowest_cost_indx];
    current_lane_ += static_cast<int8_t>(possible_missions[lowest_cost_indx]);
}

void EgoVehicle::ExecuteKeepLane()
{
    bool too_close{false};
    Vehicle vehicle_ahead;

    if (road_.GetVehicleAhead(vehicle_ahead, pos_.s, current_lane_))
    {
        const double gap = (vehicle_ahead.s - pos_.s);
        if (gap < kPrefferedFrontGap)
        {
            ChooseNextMission();

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
    else if(reference_velocity_ < road_.kSpeedLimit)
    {
        reference_velocity_ += kMaxAcceleration;
    }
}

void EgoVehicle::ExecuteLaneChange()
{
    if (road_.IsLaneTransitionCompleted(pos_.d, current_lane_));
    {
        mission_ = Mission::kKeepLane;
    }
}

void EgoVehicle::PlanMission()
{
    switch (mission_)
    {
    case Mission::kKeepLane:
    {
        ExecuteKeepLane();
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
