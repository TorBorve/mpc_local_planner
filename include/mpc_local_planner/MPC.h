#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

namespace mpc {
    struct State {
        double x;
        double y;
        double psi;
    };

    struct Input {
        double vel;
        double omega;
    };

    struct OptVariables {
        State x;
        Input u;
    };

    struct MPCReturn {
        Input u0;
        std::vector<OptVariables> mpcHorizon;
        double computeTime;
        bool success;
    };

    struct Bounds {
        OptVariables lower;
        OptVariables upper;
    };

    struct Model;
    struct Cost;
    struct Constraints;
    struct Track;

    class MPC {
    public:
        MPC() = default;
        void setPath(); // TODO define input
        MPCReturn solve(const State& x0);
        MPCReturn solve(const State& x0, const Eigen::Vector3d& coeff);
    private:
        // Bounds bounds_;
        // Model model_;
        // Cost cost_;
        // Constraints constraints_;
        // Track track_;
    };
} 