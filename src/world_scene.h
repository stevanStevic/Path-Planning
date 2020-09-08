#ifndef WORLD_SCENE_H
#define WORLD_SCENE_H

#include "ego_vehicle.h"
#include "road.h"
#include "types.h"

class WorldScene
{
  public:
    WorldScene(const WorldMap& map);

    void UpdateScene(const Vehicle& localization_data,
                     const TrajectoryPts& prev_trajectory,
                     const std::vector<std::vector<double>>& sensor_fusion);

    TrajectoryPts AdvanceEgoVehicle();

  private:
    Road road_;
    EgoVehicle ego_vehicle_;
    const WorldMap& map_;
};

#endif // WORLD_SCENE_H