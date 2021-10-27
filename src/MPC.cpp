#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"
#include "mpc_local_planner/utilities.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <fstream>

namespace mpc {

    constexpr size_t N = 30;
    constexpr double dt = 0.1;

    constexpr size_t x_start = 0;
    constexpr size_t y_start = x_start + N;
    constexpr size_t psi_start = y_start + N;
    constexpr size_t v_start = psi_start + N;
    constexpr size_t cte_start = v_start + N;
    constexpr size_t epsi_start = cte_start + N;
    constexpr size_t delta_start = epsi_start + N;
    constexpr size_t a_start = delta_start + N - 1;

    constexpr double Lf = 2.67;
    constexpr static double ref_cte = 0;
    constexpr static double ref_epsi = 0;
    constexpr static double ref_v = 10; 

    void model(State& state, const Input& u) {
        state.x += state.vel * cos(state.psi) * dt;
        state.y += state.vel * sin(state.psi) * dt;
        state.psi += state.vel * u.delta / Lf * dt;
        state.vel += u.a * dt;
    }

    class FG_eval {
    public:
        using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);
        
        FG_eval(const Eigen::Vector4d& coeffs) {
            this->coeffs = coeffs;
        }
        
        void operator()(ADvector& fg, const ADvector& vars) {
            // cost function
            fg[0] = 0;

            for (unsigned int i=0; i<N; i++)
            {
            fg[0] += 2000*CppAD::pow(vars[cte_start+i]-ref_cte, 2);
            fg[0] += 2000*CppAD::pow(vars[epsi_start+i]-ref_epsi, 2);
            fg[0] += 1*CppAD::pow(vars[v_start+i]-ref_v, 2);
            }

            // minimize the use of actuators
            for (unsigned int i=0; i<N-1; i++)
            {
            fg[0] += 25*CppAD::pow(vars[delta_start+i], 2);
            fg[0] += 25*CppAD::pow(vars[a_start+i], 2);
            //fg[0] += 700*CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
            }

            // minimize the value gap between sequential actuations
            for (unsigned int i=0; i<N-2; i++)
            {
            fg[0] += 200*CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2); 
            fg[0] += 20*CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
            }
    
            //
            // Setup Constraints
            //
            // NOTE: In this section you'll setup the model constraints.

            // Initial constraints
            //
            // We add 1 to each of the starting indices due to cost being located at
            // index 0 of `fg`.
            // This bumps up the position of all the other values.
            fg[1 + x_start] = vars[x_start];
            fg[1 + y_start] = vars[y_start];
            fg[1 + psi_start] = vars[psi_start];
            fg[1 + v_start] = vars[v_start];
            fg[1 + cte_start] = vars[cte_start];
            fg[1 + epsi_start] = vars[epsi_start];

            // The rest of the constraints
            for (unsigned int i = 0; i < N -1; i++) 
            {
            // The state at time t+1 .
            CppAD::AD<double> x1 = vars[x_start + i + 1];
            CppAD::AD<double> y1 = vars[y_start + i + 1];
            CppAD::AD<double> psi1 = vars[psi_start + i + 1];
            CppAD::AD<double> v1 = vars[v_start + i + 1];
            CppAD::AD<double> cte1 = vars[cte_start + i + 1];
            CppAD::AD<double> epsi1 = vars[epsi_start + i + 1];

            // The state at time t.
            CppAD::AD<double> x0 = vars[x_start + i];
            CppAD::AD<double> y0 = vars[y_start + i];
            CppAD::AD<double> psi0 = vars[psi_start + i];
            CppAD::AD<double> v0 = vars[v_start + i];
            CppAD::AD<double> cte0 = vars[cte_start + i];
            CppAD::AD<double> epsi0 = vars[epsi_start + i];
            // Only consider the actuation at time t.
            CppAD::AD<double> delta0 = vars[delta_start + i];
            CppAD::AD<double> a0 = vars[a_start + i];
            //if (i > 0) {   // use previous actuations (to account for latency)
            //  a0 = vars[a_start + i - 1];
            //  delta0 = vars[delta_start + i - 1];
            //}
            
            CppAD::AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
            CppAD::AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + i] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] =
                epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
            }
            return;
        }
    private:
        Eigen::Vector4d coeffs;
    };

    void MPC::calcState(State& state) const {
        state.cte = state.y - polyEval(state.x);
        state.epsi = state.psi - atan(coeffs[1] + 2 * state.x * coeffs[2] + 3 * coeffs[3] * state.x * state.x);
        return;
    }

    MPCReturn MPC::solve(const State& state){
        using Dvector = CPPAD_TESTVECTOR(double);

        auto start = std::chrono::high_resolution_clock::now();

        double x = state.x;
        double y = state.y;
        double psi = state.psi;
        double v = state.vel;
        double cte = state.cte;
        double epsi = state.epsi;

        // TODO: Set the number of model variables (includes both states and inputs).
        // For example: If the state is a 4 element vector, the actuators is a 2
        // element vector and there are 10 timesteps. The number of variables is:
        //
        // 4 * 10 + 2 * 9
        size_t n_vars = N * 6 + (N-1)*2;
        // TODO: Set the number of constraints
        size_t n_constraints = N * 6;


        // Initial value of the independent variables.
        // SHOULD BE 0 besides initial state.
        Dvector vars(n_vars);
        for (unsigned int i = 0; i < n_vars; i++) {
            vars[i] = 0;
        }

        vars[x_start] = x;
        vars[y_start] = y;
        vars[psi_start] = psi;
        vars[v_start] = v;
        vars[cte_start] = cte;
        vars[epsi_start] = epsi;

        BoundVector varBounds(n_vars, Bound::noBound());

        // upper/lower limits for delta set to -25/25
        // degrees(values in radians)
        for (unsigned int i = delta_start; i < a_start; i++) {
            varBounds[i] = Bound{-0.436332*Lf, 0.436332*Lf};
        }

        // acceleration/deceleration upper/lower limits 
        for (unsigned int i = a_start; i < n_vars; i++) {
            varBounds[i] = Bound{-1, 1};
        }


        // Lower and upper limits for the constraints
        // Should be 0 besides initial state.
        BoundVector constraintBounds(n_constraints, Bound::zeroBound());

        constraintBounds[x_start] = Bound{x, x};
        constraintBounds[y_start] = Bound{y, y};
        constraintBounds[psi_start] = Bound{psi, psi};
        constraintBounds[v_start] = Bound{v, v};
        constraintBounds[cte_start] = Bound{cte, cte};
        constraintBounds[epsi_start] = Bound{epsi, epsi};

        // object that computes objective and constraints
        FG_eval fg_eval(coeffs);

        //
        // NOTE: You don't have to worry about these options
        //
        // options for IPOPT solver
        std::string options;
        // Uncomment this if you'd like more print information
        options += "Integer print_level  0\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.
        options += "Numeric max_cpu_time          0.25\n";

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        // solve the problem
        Dvector varsLower = toCppAD(getLower(varBounds));
        Dvector varsUpper = toCppAD(getUpper(varBounds));
        Dvector constraintsLower = toCppAD(getLower(constraintBounds));
        Dvector constraintsUpper = toCppAD(getUpper(constraintBounds));
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, varsLower, varsUpper, constraintsLower,
            constraintsUpper, fg_eval, solution);

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
        ret.mpcHorizon.resize(N - 1); // TODO should be N error due to a?
        ret.u0 = Input{x[a_start], x[delta_start]};
        for (int i = 0; i < N - 1; i++){ // TODO should be N error due to a?
            State state{x[x_start + i], x[y_start + i], x[psi_start + i], x[v_start + i],
                        x[cte_start + i], x[epsi_start + i]};
            Input input{x[a_start + i], x[delta_start + i]};
            ret.mpcHorizon[i] = OptVariables{state, input};
        }
        ret.cost = solution.obj_value;
        ret.success = solution.success;
        ret.computeTime = time;
        return ret;
    }
}