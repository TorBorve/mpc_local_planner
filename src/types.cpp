#include "mpc_local_planner/types.h"

#include <unordered_map>

namespace mpc {

Mode str2Mode(const std::string &str) {
    static const std::unordered_map<std::string, Mode> table = {
        {"path_tracking", Mode::PathTracking}, {"parking", Mode::Parking}, {"slalom", Mode::Slalom}};
    auto it = table.find(str);
    if (it == table.end()) {
        return Mode::Invalid;
    }
    return it->second;
}

}  // namespace mpc

std::ostream &operator<<(std::ostream &os, const mpc::State &state) {
    os << "x: " << state.x << ", y: " << state.y << ", psi: " << state.psi << ", vel: " << state.vel << ", delta: " << state.delta
       << ", throttle: " << state.throttle;
    return os;
}

std::ostream &operator<<(std::ostream &os, const mpc::Input &input) {
    os << "deltaDot: " << input.deltaDot << ", throttleDot: " << input.throttleDot;
    return os;
}

std::ostream &operator<<(std::ostream &os, const mpc::OptVariables &optVar) {
    os << optVar.x << ", " << optVar.u;
    return os;
}