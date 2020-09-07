#ifndef WORLD_SCENE_H
#define WORLD_SCENE_H

#include "types.h"
#include "ego_vehicle.h"

class WorldScene
{
  public:
    WorldScene(const WorldMap& map) : map_{map} {}

    void UpdateScene(const Vehicle& localization_data,
                     const TrajectoryPts& prev_trajectory,
                     const std::vector<std::vector<double>>& sensor_fusion);

    TrajectoryPts AdvanceEgoVehicle();

  private:
    static constexpr double kSimulationTimeStep{0.02};

    std::vector<Vehicle> predictions_;

    EgoVehicle ego_vehicle_;

    const WorldMap& map_;
};

#endif // WORLD_SCENE_H