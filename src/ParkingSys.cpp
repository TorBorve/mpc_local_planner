#include "mpc_local_planner/ParkingSys.h"

namespace mpc {

MPCReturn ParkingSys::solve(const State &state, double pitch) {
    if (!init_) {
        return MPCReturn{};
    }
    double distSqrdToGoal = distSqrd(state.x - goal_.position.x, state.y - goal_.position.y);
    if (distSqrdToGoal < 5 * 5) {
        Acados::PointStabParams params;
        params.pitch = pitch;
        params.pRef = Point{goal_.position.x, goal_.position.y};
        params.psiRef = getYaw(goal_.orientation);
        return pointStabSolver_.solve(state, params);

    } else {
        if (updatePath_) {
            createPathToGoal(state);
            updatePath_ = false;
        }
        return pathTrackingSys_.solve(state, pitch);
    }
}

void ParkingSys::createPathToGoal(const State &state) {
    Point car{state.x, state.y};
    Point goal{goal_.position.x, goal_.position.y};
    BezierCurve bCurve{car, state.psi, goal, getYaw(goal_.orientation)};
    pathTrackingSys_.setTrack(getPath(bCurve));
}

}  // namespace mpc