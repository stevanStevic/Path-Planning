#ifndef ROAD_H
#define ROAD_H

#include "types.h"

class Road
{
  public:
    Road() = default;

    void UpdateRoadContext(const std::vector<std::vector<double>>& sensor_fusion,
                           const size_t trajectory_size);

    bool GetVehicleAhead(Vehicle& vehicle_ahead, const double car_s, const uint8_t lane) const;
    bool GetNearestVehicle(Vehicle& vehicle_ahead, const double car_s, const uint8_t lane) const;

    bool IsLaneTransitionCompleted(const double car_d, const uint8_t lane) const;
    bool IsLaneChangePossible(const int8_t lane, const Mission mission) const;

    static constexpr double kLaneWidth{4.0}; // in meters
    static constexpr double kLaneCenterOffset{kLaneWidth / 2.0}; // in meters

    static constexpr double kSimulationTimeStep{0.02};

  private:
    static constexpr uint8_t kNumOfLanes{3};

    std::vector<Vehicle> predictions_;
};

#endif // ROAD_H