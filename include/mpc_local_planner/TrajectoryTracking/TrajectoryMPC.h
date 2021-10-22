#pragma once

#include "eigen3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "mpc_local_planner/MPC.h"

namespace mpc {
    namespace nonlinear {
        namespace TrajectoryTracking {

            class MPC {
            public:
                using Dvector = CPPAD_TESTVECTOR(double);

                MPC(const Eigen::Vector4d& coeffs) : coeffs{coeffs}
                {

                }
                void setPath(const Eigen::Vector4d& newCoeffs){
                    coeffs = newCoeffs;
                }
                
                Eigen::Vector4d getPath() const {
                    return coeffs;
                }
                
                Eigen::VectorXd calcState(const State& state) const;

                MPCReturn solve(const Eigen::VectorXd& state);

                MPCReturn static toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time);
            private:
                double polyEval(double x) const {
                    return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x; 
                }
                Eigen::Vector4d coeffs;
            };
        }
    }
}