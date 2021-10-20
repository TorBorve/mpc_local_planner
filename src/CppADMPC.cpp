#include "mpc_local_planner/CppADMPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <fstream>

namespace mpc {
    namespace nonlinear {

        void diffDriveModel(State& state, const Input& input, double dt){
            state.x += input.vel * std::cos(state.psi) * dt;
            state.y += input.vel * std::sin(state.psi) * dt;
            state.psi += input.omega * dt;
            // state.psi += input.vel * std::tan(input.omega) / 1.0 * dt;
        }

        constexpr size_t N = 30;
        constexpr double dt = 0.1;

        constexpr size_t x_start = 0;
        constexpr size_t y_start = x_start + N;
        constexpr size_t psi_start = y_start + N;
        constexpr size_t v_start = psi_start + N;
        constexpr size_t omega_start = v_start + N;

        class FG_eval {
        public:
            using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);
            
            FG_eval(const State& refState) : refState{refState} {}
            
            void operator()(ADvector& fg, const ADvector& vars){
                // cost function
                fg[0] = 0;
                for(unsigned int i = 0; i < N; i++){
                    fg[0] += 1000*CppAD::pow(vars[x_start + i] - refState.x, 2);
                    fg[0] += 1000*CppAD::pow(vars[y_start + i] - refState.y, 2);
                    fg[0] += 100*CppAD::pow(vars[psi_start + i] - refState.psi, 2);
                    fg[0] += CppAD::pow(vars[v_start + i], 2);
                    fg[0] += CppAD::pow(vars[omega_start + i], 2);
                }

                fg[1 + x_start] = vars[x_start];
                fg[1 + y_start] = vars[y_start];
                fg[1 + psi_start] = vars[psi_start];

                for (unsigned int i = 0; i < N - 1; i++){
                    // The state at time t+1 .
                    CppAD::AD<double> x1 = vars[x_start + i + 1];
                    CppAD::AD<double> y1 = vars[y_start + i + 1];
                    CppAD::AD<double> psi1 = vars[psi_start + i + 1];
                    // The state at time t.
                    CppAD::AD<double> x0 = vars[x_start + i];
                    CppAD::AD<double> y0 = vars[y_start + i];
                    CppAD::AD<double> psi0 = vars[psi_start + i];

                    CppAD::AD<double> v0 = vars[v_start + i];
                    CppAD::AD<double> omega0 = vars[omega_start + i];
                    fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
                    fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
                    fg[2 + psi_start + i] = psi1 - (psi0 + omega0 * dt);
                    // fg[2 + psi_start + i] = psi1 - (psi0 + v0 * CppAD::tan(omega0) / 1.0 * dt);

                }
                return;
            }
        private:
            State refState;
        };

        MPCReturn MPC::solve(const State& state){
            auto start = std::chrono::high_resolution_clock::now();
            constexpr size_t N_vars = N*3 + N*2;
            constexpr size_t N_constraints = N*3;

            //variables state and actuators
            Dvector vars(N_vars);
            for(unsigned int i = 0; i < N_vars; i++){
                vars[i] = 0;
            }
            // initial state
            vars[x_start] = state.x;
            vars[y_start] = state.y;
            vars[psi_start] = state.psi;

            Dvector vars_lowerbound(N_vars);
            Dvector vars_upperbound(N_vars);
            
            // states don't have bounds
            for (unsigned int i = 0; i < v_start; i++){
                vars_lowerbound[i] = -1.0e19;
                vars_upperbound[i] = 1.0e19;
            }
            // bounds for velocity
            for(unsigned int i = v_start; i < omega_start; i++){
                vars_lowerbound[i] = -2;
                vars_upperbound[i] = 2;
            }
            //bounds for omega
            for(unsigned int i = omega_start; i < omega_start + N; i++){
                vars_lowerbound[i] = -3.14/4;
                vars_upperbound[i] = 3.14/4;
            }
            
            Dvector constraints_lowerbound(N_constraints);
            Dvector constraints_upperbound(N_constraints);
            for (unsigned int i = 0; i < N_constraints; i++) {
                constraints_lowerbound[i] = 0;
                constraints_upperbound[i] = 0;
            }

            constraints_lowerbound[x_start] = state.x;
            constraints_upperbound[x_start] = state.x;
            constraints_lowerbound[y_start] = state.y;
            constraints_upperbound[y_start] = state.y;
            constraints_lowerbound[psi_start] = state.psi;
            constraints_upperbound[psi_start] = state.psi;

            FG_eval fg_eval{refState};

            std::string options;
            // Uncomment this if you'd like more print information
            options += "Integer print_level  0\n";
            // NOTE: Setting sparse to true allows the solver to take advantage
            // of sparse routines, this makes the computation MUCH FASTER. If you
            // can uncomment 1 of these and see if it makes a difference or not but
            // if you uncomment both the computation time should go up in orders of
            // magnitude.
            options += "Sparse  true        forward\n";
            // options += "Sparse  true        reverse\n";
            // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
            // Change this as you see fit.
            options += "Numeric max_cpu_time          10.0\n";

            // place to return solution
            CppAD::ipopt::solve_result<Dvector> solution;

            // solve the problem
            CppAD::ipopt::solve<Dvector, FG_eval>(
                options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                constraints_upperbound, fg_eval, solution);

            if (!solution.status == CppAD::ipopt::solve_result<Dvector>::success){
                std::cout << "Error: Failed to solve nlp\n";
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            return toMPCReturn(solution, duration.count());
        }

        MPCReturn MPC::toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time){
            const auto& x = solution.x;
            MPCReturn ret;
            ret.mpcHorizon.resize(N);
            ret.u0 = Input{x[v_start], x[omega_start]};
            for (int i = 0; i < N; i++){
                State state{x[x_start + i], x[y_start + i], x[psi_start + i]};
                Input input{x[v_start + i], x[omega_start + i]};
                ret.mpcHorizon[i] = OptVariables{state, input};
            }
            ret.cost = solution.obj_value;
            ret.success = solution.success;
            ret.computeTime = time;
            return ret;
        }
    }
}