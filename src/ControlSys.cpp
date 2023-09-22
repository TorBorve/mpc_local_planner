#include "mpc_local_planner/ControlSys.h"

namespace mpc {
MPCReturn ControlSys::solve(const State &state, double pitch) {
    return pathTrackingSys_.solve(state, pitch, pathTrackingVel_);
}

std::vector<Point> ControlSys::getTrack() const {
    return pathTrackingSys_.getTrack();
}
}  // namespace mpc