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

        State(const std::array<double, 6>& arr) :
            State{arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]}
        {

        }

        std::array<double, 6> toArray() const {
            return std::array<double, 6>{x, y, psi, vel, cte, epsi};
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