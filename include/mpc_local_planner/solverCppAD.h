#ifndef MPC_SOLVER_CPPAD_H_
#define MPC_SOLVER_CPPAD_H_

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

namespace mpc
{
    class FG_eval
    {
    public:
        using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);

        FG_eval(const Eigen::Vector4d &coeffs, size_t N, double dt, double wheelbase);

        void operator()(ADvector &fg, const ADvector &vars);

    private:
        Eigen::Vector4d coeffs;
        const size_t N;
        const double dt;

        const size_t x_start;
        const size_t y_start;
        const size_t psi_start;
        const size_t v_start;
        const size_t cte_start;
        const size_t epsi_start;
        const size_t delta_start;
        const size_t a_start;

        const double Lf;
        const double ref_cte = 0;
        const double ref_epsi = 0;
        const double ref_v = 10.0;
    };
}

#endif