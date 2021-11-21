#pragma once

#include <eigen3/Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <ros/ros.h>

#include "mpc_local_planner/types.h"

namespace mpc {

    class MPC {
    public:
        using Dvector = CPPAD_TESTVECTOR(double);

        MPC(const std::vector<Point>& track, size_t N, double dt);

        void setTrack(const std::vector<Point>& newTrack) { 
            track_ = newTrack;
        }
        
        std::vector<Point> getTrack() const {
            return track_;
        }

        MPCReturn solve(const OptVariables& optVars);
        
        MPCReturn solve(const OptVariables& optVars, const Eigen::Vector4d& coeffs);

        MPCReturn toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time);
        
        void model(OptVariables& optVars, const Input& u);
    private:
        void calcCoeffs(const State& state, double& rotation, Eigen::Vector4d& coeffs) const;

        Eigen::Vector4d interpolate(const State& state, double rotation, size_t start, size_t end, double& cost) const;
        
        void calcState(State& state, const Eigen::Vector4d& coeffs) const;

        double polyEval(double x, const Eigen::Vector4d& coeffs) const {
            return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x; 
        }

        void getTrackSection(size_t& start, size_t& stop, const State& state) const;

        std::vector<Point> track_;

        ros::Publisher trackPub_;
        ros::Publisher mpcPathPub_;
        ros::Publisher polynomPub_;

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
