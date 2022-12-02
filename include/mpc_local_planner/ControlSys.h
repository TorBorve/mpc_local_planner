#pragma once

#include "mpc_local_planner/ParkingSys.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ControlSys {
   public:
    ControlSys(double pathTrackingVel, double parkingVel)
        : parkingSys_{parkingVel}, pathTrackingVel_{pathTrackingVel} {}

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

    void setMode(const Mode &mode) {
        mode_ = mode;
        parkingSys_.setMode(mode);
    }

    Mode getMode() const { return mode_; }
    /// @brief set the reference pose for parking system.
    /// @param[in] pose the pose we want to get to.
    void setRefPose(const geometry_msgs::msg::Pose &pose) { parkingSys_.setRefPose(pose); }

    /// @brief get the reference pose for the parking system.
    geometry_msgs::msg::Pose getRefPose() const { return parkingSys_.getRefPose(); }

   private:
    /// @brief the parking system class
    ParkingSys parkingSys_;

    /// @brief the path tracking system class
    PathTrackingSys pathTrackingSys_ =
        PathTrackingSys{util::getTestTrack()};  // TODO remove use of getTestTrack

    /// @brief mode of the control system.
    Mode mode_ = Mode::Invalid;

    /// @brief the desired speed of the car when we are doing path tracking
    double pathTrackingVel_;
};

}  // namespace mpc