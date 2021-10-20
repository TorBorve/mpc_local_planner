#pragma once

#include "eigen3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "mpc_local_planner/MPC.h"

namespace mpc {
    namespace nonlinear {

        void diffDriveModel(State& state, const Input& input, double dt);

        class MPC {
        public:
            using Dvector = CPPAD_TESTVECTOR(double);

            MPC(const State& refstate) : refState{refstate}
            {

            }
            void setRefState(const State& newRefState){
                refState = newRefState;
            }
            
            State getRefState() const {
                return refState;
            }
            MPCReturn solve(const State& state);

            MPCReturn static toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time);
        private:
            State refState;
        };
    }
}