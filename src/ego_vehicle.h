#ifndef EGO_VEHICLE_H
#define EGO_VEHICLE_H

#include <vector>
#include <stdint.h>

#include "types.h"

class EgoVehicle
{
  public:
    TrajectoryPts GenerateTrajectory(const WorldMap& map);
    // void GenerateAnchorPts();

    void UpdatePosition(const Vehicle& localization_data, const TrajectoryPts& prev_trajectory);
    void PlanMission(const std::vector<Vehicle>& predictions);

  private:
    void ExecuteKeepLane(const std::vector<Vehicle>& predictions);
    void UpdateExistingTrajectory(const TrajectoryPts& prev_trajectory);
    bool GetVehicleAhead(const std::vector<Vehicle>& predictions, uint8_t current_lane);

    TrajectoryPts trajectory_;

    uint8_t current_lane_{1};
    static constexpr double kLaneWidth{4.0}; // in meters
    static constexpr double kLaneCenter{kLaneWidth / 2.0}; // in meters

    static constexpr double kPredictionTarget{30.0}; // in meters

    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double velocity_;
    double end_path_s_;
    double end_path_d_;

    double reference_velocity_{59.5};

    Mission mission_;
};

#endif // EGO_VEHICLE_H