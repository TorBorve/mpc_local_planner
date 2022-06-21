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
    ParkingSys(double refVel) : refVel_{refVel} {}

    /// @brief constructor for parking system.
    /// @param[in] goal where we want to park
    ParkingSys(const geometry_msgs::Pose &goal, double refVel) : refVel_{refVel}, goal_{goal}, init_{true} {
    }

    /// @brief solve function
    /// @param[in] state the state of the car
    /// @param[in] pitch the pitch of the car
    MPCReturn solve(const State &state, double pitch);

    /// @brief set the reference pose for the car. This is where we want to park.
    /// @param[in] pose where we want to park.
    void setRefPose(const geometry_msgs::Pose &pose);

    /// @brief get the current desired trajectory
    /// @return track for the desired trajectory
    std::vector<Point> getTrack() const {
        return pathTrackingSys_.getTrack();
    }

    /// @brief get the reference pose. This is where we want to park.
    geometry_msgs::Pose getRefPose() const {
        return goal_;
    }

   private:
    /// @brief calculate a smooth path from your position to the parking spot.
    /// @param[in] state the state of the car.
    void createPathToGoal();

    /// @brief solver for point stabilization (parking)
    Acados::PointStab pointStabSolver_ = Acados::PointStab{State{0, 0, 0, 0, 0, 0}};

    /// @brief path tracking system used to get close to the parking spot
    PathTrackingSys pathTrackingSys_ = PathTrackingSys{util::getTestTrack()};

    /// @brief the goal position. Where we want to park.
    geometry_msgs::Pose goal_;

    /// @brief if the system has been initialized
    bool init_ = false;

    /// @brief the state where the start position of the car should be
    State startState_;

    /// @brief atomic bool for indicating if we need to update the path.
    std::atomic<bool> updatePath_{false};

    /// @brief atomic bool for indicating if we need to update the start position of the path.
    std::atomic<bool> updateStart_{false};

    /// @brief mutex for ensuring no problems with data races and so on when seting updatePath_ and updateStart_
    std::mutex m;

    double refVel_;
};
}  // namespace mpc