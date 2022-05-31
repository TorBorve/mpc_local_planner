#pragma once

#include "mpc_local_planner/ParkingSys.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ControlSys {
   public:
   /// @brief modes the control system can be in.
    enum class Mode { PathTracking,
                      Parking };
    ControlSys() = default;

    /// @brief solve function
    /// @param[in] state the state of the car
    /// @param[in] pith the pitch of the car. Indicates if the are is driving up/down hill.
    MPCReturn solve(const State &state, double pitch) {
        if (mode_ == Mode::Parking) {
            return parkingSys_.solve(state, pitch);
        } else {
            return pathTrackingSys_.solve(state, pitch, 2.0);
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
        }
        return pathTrackingSys_.getTrack();
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
    PathTrackingSys pathTrackingSys_ = PathTrackingSys{getTestTrack()};

    /// @brief mode of the control system.
    Mode mode_ = Mode::PathTracking;
};

}  // namespace mpc