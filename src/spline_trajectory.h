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
    static constexpr double kTrajectoryHorizon{30.0};
};

#endif // SPLINE_TRAJECTORY_H