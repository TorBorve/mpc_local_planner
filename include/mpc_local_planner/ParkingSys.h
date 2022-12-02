#pragma once

#include <atomic>
#include <geometry_msgs/msg/pose.hpp>

#include "mpc_local_planner/AcadosPointStab.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/types.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

class ParkingSys {
   public:
    ParkingSys(double refVel) : refVel_{refVel} {}

    /// @brief constructor for parking system.
    /// @param[in] goal where we want to park
    ParkingSys(const geometry_msgs::msg::Pose &goal, double refVel) : goal_{goal}, init_{true}, refVel_{refVel} {}

    /// @brief solve function
    /// @param[in] state the state of the car
    /// @param[in] pitch the pitch of the car
    MPCReturn solve(const State &state, double pitch);

    /// @brief set the reference pose for the car. This is where we want to park.
    /// @param[in] pose where we want to park.
    void setRefPose(const geometry_msgs::msg::Pose &pose);

    /// @brief get the current desired trajectory
    /// @return track for the desired trajectory
    std::vector<Point> getTrack() const { return pathTrackingSys_.getTrack(); }

    /// @brief get the reference pose. This is where we want to park.
    geometry_msgs::msg::Pose getRefPose() const { return goal_; }

    /// @brief set the mode of the parking system. Either parking or Slalom
    /// @param[in] mode the new mode wanted.
    void setMode(Mode mode) {
        if (mode == Mode::Parking || mode == Mode::Slalom) {
            mode_ = mode;
        }
    }

    /// @brief get the mode of the parking system.
    /// @return the current mode.
    Mode getMode() const { return mode_; }

   private:
    /// @brief calculate a smooth path from your position to the parking spot.
    /// @param[in] state the state of the car.
    void createPathToGoal();

    /// @brief solver for point stabilization (parking)
    Acados::PointStab pointStabSolver_ = Acados::PointStab{State{0, 0, 0, 0, 0, 0}};

    /// @brief path tracking system used to get close to the parking spot
    PathTrackingSys pathTrackingSys_ = PathTrackingSys{util::getTestTrack()};

    /// @brief the goal position. Where we want to park.
    geometry_msgs::msg::Pose goal_;

    /// @brief if the system has been initialized
    bool init_ = false;

    /// @brief the state where the start position of the car should be
    State startState_;

    /// @brief bool for indicating if we need to update the path.
    bool updatePath_{false};

    /// @brief bool for indicating if we need to update the start position of the path.
    bool updateStart_{false};

    /// @brief the desired velocity when driving to the goal.
    double refVel_;

    /// @brief mode of the parkin system. Either slalom or parking
    Mode mode_ = Mode::Invalid;
};
}  // namespace mpc