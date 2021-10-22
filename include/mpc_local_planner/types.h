#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

namespace mpc {
    struct State {
        double x;
        double y;
        double psi;
        double vel;
        double cte;
        double epsi;
    };

    struct Input {
        double a;
        double delta;
    };

    struct OptVariables {
        State x;
        Input u;
    };

    struct MPCReturn {
        Input u0;
        std::vector<OptVariables> mpcHorizon;
        double computeTime;
        double cost;
        bool success;
    };
}