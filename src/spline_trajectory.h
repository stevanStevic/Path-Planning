#ifndef SPLINE_TRAJECTORY_H
#define SPLINE_TRAJECTORY_H

#include "types.h"

class EgoVehicle;
class Road;

class SplineTrajectory
{
  public:
    static const TrajectoryPts GenerateTrajectory(const EgoVehicle& vehicle,
                                                  const Road& road,
                                                  const WorldMap& map);

  private:
    static TrajectoryPts GenerateAnchorWaypoints(const EgoVehicle& ego_vehicle,
                                                 const Road& road,
                                                 const WorldMap& map,
                                                 double& ref_x,
                                                 double& ref_y,
                                                 double& ref_yaw);

    static void SplineConnectWaypoints(TrajectoryPts& new_trajectory,
                                       const EgoVehicle& ego_vehicle,
                                       const Road& road,
                                       double& ref_x,
                                       double& ref_y,
                                       double& ref_yaw);

    static constexpr double kTrajectoryHorizon{30.0};
};

#endif // SPLINE_TRAJECTORY_H