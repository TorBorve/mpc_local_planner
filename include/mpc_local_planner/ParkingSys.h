#pragma once

#include <atomic>

#include <geometry_msgs/Pose.h>

#include "mpc_local_planner/AcadosPointStab.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ParkingSys {
   public:
    ParkingSys() = default;

    ParkingSys(const geometry_msgs::Pose &goal) : goal_{goal}, init_{true} {
    }

    MPCReturn solve(const State &state, double pitch);

    void setRefPose(const geometry_msgs::Pose &pose) {
        goal_ = pose;
        init_ = true;
        updatePath_ = true;
    }

    /// @brief get the current desired trajectory
    /// @return track for the desired trajectory
    std::vector<Point> getTrack() const {
        return pathTrackingSys_.getTrack();
    }

    geometry_msgs::Pose getRefPose() const {
        return goal_;
    }

   private:
    void createPathToGoal(const State &state);
    Acados::PointStab pointStabSolver_ = Acados::PointStab{State{0, 0, 0, 0, 0, 0}};
    PathTrackingSys pathTrackingSys_ = PathTrackingSys{getTestTrack()};
    geometry_msgs::Pose goal_;

    bool init_ = false;
    std::atomic<bool> updatePath_{false};
};
}  // namespace mpc