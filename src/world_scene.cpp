#include "world_scene.h"

#include <cmath>

TrajectoryPts WorldScene::AdvanceEgoVehicle()
{
    ego_vehicle_.PlanMission(predictions_);
    return ego_vehicle_.GenerateTrajectory(map_);
}

void WorldScene::UpdateScene(const Vehicle& localization_data,
                             const TrajectoryPts& prev_trajectory,
                             const std::vector<std::vector<double>>& sensor_fusion)
{
    // Update ego vehicle position and trajectory based on sensed data.
    // Data includes output of localization module and trajectory pts left to drive
    ego_vehicle_.UpdatePosition(localization_data, prev_trajectory);

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

        const size_t trajectory_pts_left = prev_trajectory.x_pts.size();

        // Predict where the vehicle will be on S axis
        const double end_path_s = s + (velocity * kSimulationTimeStep * trajectory_pts_left);

        // Unimportant for predicted vehicles
        const double yaw{0};
        const double end_path_d{d};    // Stays in the same lane

        const Vehicle vehicle{x, y, s, d, yaw, velocity, end_path_s, end_path_d};

        predictions_.push_back(vehicle);
    }
}