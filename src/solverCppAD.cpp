#include "mpc_local_planner/solverCppAD.h"

namespace mpc
{
    FG_eval::FG_eval(const Eigen::Vector4d &coeffs, size_t N, double dt,
                     double wheelbase) : coeffs{coeffs}, N{N}, dt{dt}, Lf{wheelbase}, x_start{0}, y_start{N},
                                         psi_start{2 * N}, v_start{3 * N}, cte_start{4 * N}, epsi_start{5 * N},
                                         delta_start{6 * N}, a_start{7 * N - 1}
    {
    }

    void FG_eval::operator()(ADvector &fg, const ADvector &vars)
    {
        // cost function
        fg[0] = 0;

        for (unsigned int i = 0; i < N; i++)
        {
            fg[0] += 500 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
            fg[0] += 2000 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
            fg[0] += 5 * CppAD::pow(vars[v_start + i] - ref_v, 2);
        }

        // minimize the use of actuators
        for (unsigned int i = 0; i < N - 1; i++)
        {
            fg[0] += 25 * CppAD::pow(vars[delta_start + i], 2);
            fg[0] += 25 * CppAD::pow(vars[a_start + i], 2);
            // fg[0] += 700*CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
        }

        // minimize the value gap between sequential actuations
        for (unsigned int i = 0; i < N - 2; i++)
        {
            fg[0] += 200 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += 20 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
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
        for (unsigned int i = 0; i < N - 1; i++)
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
            // if (i > 0) {   // use previous actuations (to account for latency)
            //   a0 = vars[a_start + i - 1];
            //   delta0 = vars[delta_start + i - 1];
            // }

            CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
            CppAD::AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

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
            fg[2 + psi_start + i] = psi1 - (psi0 + v0 * CppAD::tan(delta0) / Lf * dt);
            // fg[2 + v_start + i] = v1 - v0; //v1 - (v0 + a0 * dt);
            fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + i] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] =
                epsi1 - ((psi0 - psides0) + v0 * CppAD::tan(delta0) / Lf * dt);
        }

        for (unsigned int i = delta_start; i < delta_start + N - 2; i++)
        {
            CppAD::AD<double> delta0 = vars[i];
            CppAD::AD<double> delta1 = vars[i + 1];
            fg[1 + i] = delta1 - delta0;
        }
        return;
    }
}