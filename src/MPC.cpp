#include "mpc_local_planner/MPC.h"
#include <chrono>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

namespace mpc {
    class FG_eval {
    public:
        Eigen::Vector3d coeff;
        FG_eval(const Eigen::Vector3d& coeff) : coeff{coeff} {

        }

        typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
        void operator()(ADvector& fg, const ADvector& vars){
            fg[0] = 0;
        }
    };

    MPCReturn MPC::solve(const State& x0){
        auto time_start = std::chrono::high_resolution_clock::now();

        MPCReturn solution;
        auto time_end = std::chrono::high_resolution_clock::now();
        solution.computeTime = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
        return solution;
    }

    MPCReturn solve(const State& x0, const Eigen::Vector3d& coeff){
        constexpr size_t N = 10;
        constexpr double dt = 0.1;

        State refState{0, 0, 0, 10, 0, 0};

    }
}