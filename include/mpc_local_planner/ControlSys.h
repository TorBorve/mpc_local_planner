#pragma once

#include "mpc_local_planner/ParkingSys.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ControlSys {
   public:
    enum class Mode { PathTracking,
                      Parking };
    ControlSys() = default;

    MPCReturn solve(const State &state, double pitch) {
        if (mode_ == Mode::Parking) {
            return parkingSys_.solve(state, pitch);
        } else {
            return pathTrackingSys_.solve(state, pitch);
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
        return pathTrackingSys_.getTrack();
    }

    void setRefPose(const geometry_msgs::Pose &pose) {
        parkingSys_.setRefPose(pose);
    }

    geometry_msgs::Pose getRefPose() const {
        return parkingSys_.getRefPose();
    }

   private:
    ParkingSys parkingSys_;
    PathTrackingSys pathTrackingSys_ = PathTrackingSys{getTestTrack()};

    Mode mode_ = Mode::PathTracking;
};

}  // namespace mpc