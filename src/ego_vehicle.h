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
    bool IsLaneTransitionCompleted() const;
    void ExecuteLaneChange();
    bool IsLaneChangePossible(const Mission mission) const;
    std::vector<Mission> GetPossibleMissions();
    void ChooseNextMission(const std::vector<Vehicle>& predictions);
    void ExecuteKeepLane(const std::vector<Vehicle>& predictions);
    void UpdateExistingTrajectory(const TrajectoryPts& prev_trajectory);
    bool GetVehicleAhead(const std::vector<Vehicle>& predictions, Vehicle& vehicle_ahead, uint8_t ego_lane);
    bool GetNearestVehicle(const std::vector<Vehicle>& predictions, Vehicle& vehicle_ahead, uint8_t ego_lane);

    TrajectoryPts trajectory_;

    uint8_t current_lane_{1};
    static constexpr double kLaneWidth{4.0}; // in meters
    static constexpr double kLaneCenterOffset{kLaneWidth / 2.0}; // in meters
    static constexpr uint8_t kNumOfLanes{3};

    static constexpr double kPredictionHorizon{30.0}; // in meters

    static constexpr double kPrefferedFrontGap{30.0};

    static constexpr double kSpeedLimit{49.5};
    static constexpr double kTakeoverLimit{45.0};

    static constexpr double kMaxAcceleration{0.224};

    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double velocity_;
    double end_path_s_;
    double end_path_d_;

    double reference_velocity_{0.0};

    Mission mission_{Mission::kKeepLane};
};

#endif // EGO_VEHICLE_H