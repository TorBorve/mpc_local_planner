#pragma once

#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"

#include <nav_msgs/Path.h>

#include <cppad/cppad.hpp>

namespace mpc{
    geometry_msgs::Pose toMsg(const State& state);
    
    CPPAD_TESTVECTOR(double) toCppAD(const std::vector<double>& vec);

    void logSolution(const MPCReturn& solution, const State& curState, const std::string& filename);

    Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

    inline double distSqrd(double dx, double dy) {
        return dx * dx + dy * dy;
    }

    std::vector<Point> getTestTrack();

    nav_msgs::Path getPathMsg(const MPCReturn& solution);

    nav_msgs::Path getPathMsg(const Eigen::Vector4d& coeffs);

    nav_msgs::Path getPathMsg(const std::vector<Point>& track);
}