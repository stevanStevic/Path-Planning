#include "world_scene.h"

#include "spline_trajectory.h"

WorldScene::WorldScene(const WorldMap& map) : map_{map}, ego_vehicle_{road_}
{
    // Nothing to be done
}

void WorldScene::UpdateScene(const Vehicle& localization_data,
                             const TrajectoryPts& prev_trajectory,
                             const std::vector<std::vector<double>>& sensor_fusion)
{
    // Update ego vehicle position and trajectory based on sensed data.
    // Data includes output of localization module and trajectory pts left to drive
    ego_vehicle_.UpdatePosition(localization_data, prev_trajectory);

    // Update road and predictions for positions of non-ego vehicles
    road_.UpdateRoadContext(sensor_fusion, prev_trajectory.x_pts.size());
}

TrajectoryPts WorldScene::AdvanceEgoVehicle()
{
    ego_vehicle_.PlanMission(road_);

    TrajectoryPts new_trajectory = SplineTrajectory::GenerateTrajectory(ego_vehicle_, road_, map_);

    ego_vehicle_.UpdateExistingTrajectory(new_trajectory);

    return new_trajectory;
}