#include "mpc_local_planner/ParkingSys.h"

#include "mpc_local_planner/utilities.h"

namespace mpc {

MPCReturn ParkingSys::solve(const State &state, double pitch) {
    if (!init_) {
        return MPCReturn{};
    }
    Acados::PointStabParams params;
    params.pitch = pitch;
    params.pRef = Point{refPose_.position.x, refPose_.position.y};
    params.psiRef = getYaw(refPose_.orientation);
    return pointStabSolver_.solve(state, params);
}

}