#include "road.h"

#include <cmath>
#include <limits>

void Road::UpdateRoadContext(const std::vector<std::vector<double>>& sensor_fusion,
                             const size_t trajectory_size)
{
    // Predict non-ego vehicle positions
    predictions_.clear();

    for (size_t i = 0; i < sensor_fusion.size(); ++i)
    {
        const double id = sensor_fusion[i][0];
        const double x = sensor_fusion[i][1];
        const double y = sensor_fusion[i][2];
        const double vx = sensor_fusion[i][3];
        const double vy = sensor_fusion[i][4];

        const double velocity = std::sqrt(vx*vx + vy*vy);

        const double s = sensor_fusion[i][5];
        const float d = sensor_fusion[i][6];

        // Predict where the vehicle will be on S axis
        const double end_path_s = s + (velocity * kSimulationTimeStep * trajectory_size);

        // Unimportant for predicted vehicles
        const double yaw{0};
        const double end_path_d{d};    // Stays in the same lane

        const Vehicle vehicle{x, y, s, d, yaw, velocity, end_path_s, end_path_d};

        predictions_.push_back(vehicle);
    }
}

bool Road::GetVehicleAhead(Vehicle& vehicle_ahead, const double car_s, const uint8_t lane) const
{
    bool found_vehicle{false};
    double closest_vehicle_distance = std::numeric_limits<double>::max();

    for (const auto& vehicle : predictions_)
    {
        const float d = vehicle.d;

        // Check if detected vehicle is in lane of ego vehicle
        if ((d > lane * kLaneWidth) && (d < (lane * kLaneWidth + kLaneWidth)))
        {
            // Check if it is closer
            if (vehicle.s > car_s && vehicle.s < closest_vehicle_distance)
            {
                closest_vehicle_distance = vehicle.s;
                vehicle_ahead = vehicle;
                found_vehicle = true;
            }
        }
    }

    return found_vehicle;
}

bool Road::GetNearestVehicle(Vehicle& vehicle_ahead, const double car_s, const uint8_t lane) const
{
    bool found_vehicle{false};
    double closest_vehicle_distance = std::numeric_limits<double>::max();

    for (const auto& vehicle : predictions_)
    {
        const float d = vehicle.d;
        // Check if detected vehicle is in lane of ego vehicle
        if ((d > lane * kLaneWidth) && (d < (lane * kLaneWidth + kLaneWidth)))
        {
            // Find closest of front and back
            double distance_diff = std::fabs(vehicle.s - car_s);
            if (distance_diff < closest_vehicle_distance)
            {
                closest_vehicle_distance = distance_diff;
                vehicle_ahead = vehicle;
                found_vehicle = true;
            }
        }
    }

    return found_vehicle;
}

bool Road::IsLaneTransitionCompleted(const double car_d, const uint8_t lane) const
{
    const double reference_lane_center = kLaneCenterOffset + lane * kLaneWidth;

    const double diff_to_lane_center = std::fabs(reference_lane_center - car_d);

    return diff_to_lane_center < 0.2;
}

bool Road::IsLaneChangePossible(const int8_t lane, const Mission mission) const
{
    const int8_t new_lane = lane + static_cast<int8_t>(mission);

    return (new_lane >= 0) && (new_lane < kNumOfLanes);
}