#include "mpc_local_planner/types.h"

std::ostream &operator<<(std::ostream &os, const mpc::State &state) {
    os << "x: " << state.x << ", y: " << state.y << ", psi: " << state.psi << ", vel: " << state.vel << ", delta: " << state.delta << ", throttle: " << state.throttle;
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