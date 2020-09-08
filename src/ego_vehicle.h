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
    void PlanMission(const Road& road);

  private:
    void ExecuteKeepLane(const Road& road);
    void ExecuteLaneChange(const Road& road);

    void ChooseNextMission(const Road& road);

    std::vector<Mission> GetPossibleMissions() const;



    static constexpr double kPrefferedFrontGap{30.0};

    static constexpr double kSpeedLimit{49.5};
    static constexpr double kTakeoverLimit{45.0};

    static constexpr double kMaxAcceleration{0.224};

    Mission mission_{Mission::kKeepLane};

    const Road& road_;

  public:
    uint8_t current_lane_{1};

    TrajectoryPts trajectory_;

    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double velocity_;
    double end_path_s_;
    double end_path_d_;

    double reference_velocity_{0.0};

};

#endif // EGO_VEHICLE_H