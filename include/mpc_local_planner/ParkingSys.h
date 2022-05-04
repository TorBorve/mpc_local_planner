#pragma once

#include <geometry_msgs/Pose.h>

#include "mpc_local_planner/AcadosPointStab.h"
#include "mpc_local_planner/types.h"

namespace mpc {

class ParkingSys {
   public:
    ParkingSys() = default;

    ParkingSys(const geometry_msgs::Pose &pose) : refPose_{pose}, init_{true} {
    }

    MPCReturn solve(const State &state, double pitch);

    void setRefPose(const geometry_msgs::Pose &pose) {
        refPose_ = pose;
        init_ = true;
    }

    geometry_msgs::Pose getRefPose() const {
        return refPose_;
    }

   private:
    Acados::PointStab pointStabSolver_ = Acados::PointStab{State{0, 0, 0, 0, 0, 0}};
    geometry_msgs::Pose refPose_;

    bool init_ = false;
};
}  // namespace mpc