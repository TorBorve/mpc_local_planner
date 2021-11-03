#pragma once

#include "eigen3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "mpc_local_planner/types.h"

namespace mpc {

    class MPC {
    public:
        using Dvector = CPPAD_TESTVECTOR(double);

        MPC(const std::vector<Point>& track, size_t N, double dt) : 
            track{track}, N{N}, dt{dt}, x_start{0}, y_start{N},
            psi_start{2*N}, v_start{3*N}, cte_start{4*N}, epsi_start{5*N},
            delta_start{6*N}, a_start{7*N - 1} // -1 due to N-1 actuator variables
        {

        }

        // void setPath(const Eigen::Vector4d& newCoeffs){
        //     coeffs = newCoeffs;
        // }
        
        // Eigen::Vector4d getPath() const {
        //     return coeffs;
        // }

        void setTrack(const std::vector<Point>& newTrack) { 
            track = newTrack;
        }
        
        std::vector<Point> getTrack() const {
            return track;
        }

        MPCReturn solve(const State& state);
        
        MPCReturn solve(const State& state, const Eigen::Vector4d& coeffs);

        MPCReturn toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time);
        
        void model(State& state, const Input& u);
    private:
        Eigen::Vector4d calcCoeffs(const State& state) const;
        
        void calcState(State& state, const Eigen::Vector4d& coeffs) const;

        double polyEval(double x, const Eigen::Vector4d& coeffs) const {
            return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x; 
        }

        void getTrackSection(size_t& start, size_t& stop, const State& state) const;
        // Eigen::Vector4d coeffs;
        std::vector<Point> track;

        size_t N;
        double dt;
        constexpr static double Lf = 2.67;

        const size_t x_start;
        const size_t y_start;
        const size_t psi_start;
        const size_t v_start;
        const size_t cte_start;
        const size_t epsi_start;
        const size_t delta_start;
        const size_t a_start;
    };

}
