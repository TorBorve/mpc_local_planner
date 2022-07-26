#pragma once

#include "mpc_local_planner/ParkingSys.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ControlSys {
   public:
       ControlSys(double pathTrackingVel, double parkingVel) : pathTrackingVel_{pathTrackingVel}, parkingSys_{parkingVel} {}

    /// @brief solve function
    /// @param[in] state the state of the car
    /// @param[in] pitch the pitch of the car. Indicates if the car is driving up/down hill.
    MPCReturn solve(const State &state, double pitch) {
        if (mode_ == Mode::Parking || mode_ == Mode::Slalom) {
            return parkingSys_.solve(state, pitch);
        } else if (mode_ == Mode::PathTracking) {
            return pathTrackingSys_.solve(state, pitch, pathTrackingVel_);
        } else {
            throw std::runtime_error{"Mode of control system not set. use the function setMode to update mode"};
        }
    }

    /// @brief update track variable for desired trajectory
    /// @param[in] newTrack the new desired trajectory
    void setTrack(const std::vector<Point> &newTrack) {
        pathTrackingSys_.setTrack(newTrack);
    }

    /// @brief get the current desired trajectory
    /// @return track for the desired trajectory
    std::vector<Point> getTrack() const {
        if (mode_ == Mode::Parking) {
            return parkingSys_.getTrack();
        } else if (mode_ == Mode::PathTracking) {
            return pathTrackingSys_.getTrack();
        } else {
            return std::vector<Point>{};
        }
    }

    void setMode(const Mode &mode) {
        mode_ = mode;
        parkingSys_.setMode(mode);
    }

    Mode getMode() const {
        return mode_;
    }
    /// @brief set the reference pose for parking system.
    /// @param[in] pose the pose we want to get to.
    void setRefPose(const geometry_msgs::Pose &pose) {
        parkingSys_.setRefPose(pose);
    }

    /// @brief get the reference pose for the parking system.
    geometry_msgs::Pose getRefPose() const {
        return parkingSys_.getRefPose();
    }

   private:
    /// @brief the parking system class
    ParkingSys parkingSys_;

    /// @brief the path tracking system class
    PathTrackingSys pathTrackingSys_ = PathTrackingSys{util::getTestTrack()};  // TODO remove use of getTestTrack

    /// @brief mode of the control system.
    Mode mode_ = Mode::Invalid;

    double pathTrackingVel_;
};

}  // namespace mpc