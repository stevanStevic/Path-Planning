#ifndef EGO_VEHICLE_H
#define EGO_VEHICLE_H

#include <vector>
#include <stdint.h>

#include "types.h"

class Road;

class EgoVehicle
{
  public:
    EgoVehicle(const Road& road) : road_(road) {}

    void UpdateExistingTrajectory(const TrajectoryPts& trajectory);
    void UpdatePosition(const Vehicle& localization_data, const TrajectoryPts& prev_trajectory);

    void PlanMission();

  private:
    void ExecuteKeepLane();
    void ExecuteLaneChange();

    void ChooseNextMission();
    std::vector<Mission> GetPossibleMissions() const;

    const int GetLowestCostIndx(const std::vector<double>& costs) const;
    const double CalculateMissionCost(const Mission mission) const;

    static constexpr double kPrefferedFrontGap{30.0};
    static constexpr double kMaxAcceleration{0.224};

    Mission mission_{Mission::kKeepLane};
    const Road& road_;

  public:
    uint8_t current_lane_{1};
    double reference_velocity_{0.0};

    TrajectoryPts trajectory_;
    Vehicle pos_;
};

#endif // EGO_VEHICLE_H