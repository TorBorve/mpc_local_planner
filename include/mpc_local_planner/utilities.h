#pragma once

#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"

#include <nav_msgs/Path.h>

#include <cppad/cppad.hpp>

namespace mpc{
    geometry_msgs::Pose toMsg(const State& state);
    
    nav_msgs::Path getPathMsg(const MPCReturn& solution);

    CPPAD_TESTVECTOR(double) toCppAD(const std::vector<double>& vec);

    void logSolution(const MPCReturn& solution, const State& curState, const std::string& filename);
}