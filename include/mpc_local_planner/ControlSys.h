#pragma once

#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ControlSys {
   public:
    ControlSys(double pathTrackingVel)
        : pathTrackingVel_{pathTrackingVel} {}

    /// @brief solve function
    /// @param[in] state the state of the car
    /// @param[in] pitch the pitch of the car. Indicates if the car is driving up/down hill.
    MPCReturn solve(const State &state, double pitch);

    /// @brief update track variable for desired trajectory
    /// @param[in] newTrack the new desired trajectory
    void setTrack(const std::vector<Point> &newTrack) { pathTrackingSys_.setTrack(newTrack); }

    /// @brief get the current desired trajectory
    /// @return track for the desired trajectory
    std::vector<Point> getTrack() const;

   private:
    /// @brief the path tracking system class
    PathTrackingSys pathTrackingSys_ =
        PathTrackingSys{util::getTestTrack()};  // TODO remove use of getTestTrack

    /// @brief the desired speed of the car when we are doing path tracking
    double pathTrackingVel_;
};

}  // namespace mpc