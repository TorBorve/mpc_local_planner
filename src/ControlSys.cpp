#include "mpc_local_planner/ControlSys.h"

namespace mpc {
MPCReturn ControlSys::solve(const State &state, double pitch) {
    if (mode_ == Mode::Parking || mode_ == Mode::Slalom) {
        return parkingSys_.solve(state, pitch);
    } else if (mode_ == Mode::PathTracking) {
        return pathTrackingSys_.solve(state, pitch, pathTrackingVel_);
    } else {
        throw std::runtime_error{
            "Mode of control system not set. use the function setMode to update mode"};
    }
}

std::vector<Point> ControlSys::getTrack() const {
        if (mode_ == Mode::Parking) {
            return parkingSys_.getTrack();
        } else if (mode_ == Mode::PathTracking) {
            return pathTrackingSys_.getTrack();
        } else {
            return std::vector<Point>{};
        }
    }
}  // namespace mpc