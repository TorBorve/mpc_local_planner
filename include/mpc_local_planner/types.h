#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

namespace mpc {
    struct State {
        State() = default;

        State(double x, double y, double psi, double vel, double cte, double epsi) :
            x{x}, y{y}, psi{psi}, vel{vel}, cte{cte}, epsi{epsi}
        {

        }

        double x;
        double y;
        double psi;
        double vel;
        double cte;
        double epsi;
    };

    struct Input {
        Input() = default;

        Input(double a, double delta) :
            a{a}, delta{delta}
        {

        }

        double a;
        double delta;
    };

    struct OptVariables {
        OptVariables() = default;

        OptVariables(const State& x, const Input& u) :
            x{x}, u{u}
        {

        }

        State x;
        Input u;
    };

    struct MPCReturn {
        MPCReturn() = default;

        MPCReturn(const Input& u0, const std::vector<OptVariables>& mpcHorizon,
                double computeTime, double cost, bool success) :
                u0{u0}, mpcHorizon{mpcHorizon}, computeTime{computeTime},
                cost{cost}, success{success}
        {

        }

        MPCReturn(const std::vector<OptVariables>& mpcHorizon,
                double computeTime, double cost, bool success) :
                MPCReturn{mpcHorizon.at(0).u, mpcHorizon, computeTime, cost, success}
        {
            
        }

        Input u0;
        std::vector<OptVariables> mpcHorizon;
        double computeTime;
        double cost;
        bool success;
    };
}