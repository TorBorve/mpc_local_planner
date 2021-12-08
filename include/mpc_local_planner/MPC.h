#pragma once

#include <eigen3/Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <ros/ros.h>

#include "mpc_local_planner/types.h"

namespace mpc {

    /// @brief MPC class for car
    class MPC {
    public:

        /// @brief alias for CppAD vector
        using Dvector = CPPAD_TESTVECTOR(double);

        /// @brief constructor for MPC class
        /// @param[in] track vector with points that define desired trajectory
        /// @param[in] N number of steps in simulation
        /// @param[in] dt time increment for model
        MPC(const std::vector<Point>& track, size_t N, double dt);

        /// @brief update track variable for desired trajectory
        /// @param[in] newTrack the new desired trajectory
        void setTrack(const std::vector<Point>& newTrack) { 
            track_ = newTrack;
        }
        
        /// @brief get the current desired trajectory
        /// @return track for the desired trajectory
        std::vector<Point> getTrack() const {
            return track_;
        }

        /// @brief solve function for mpc. Solves the nlp with state as given.
        /// @param[in] optVars the state of the car. (position, velocity, steering angle, ...)
        /// @return solution from mpc. See definition of MPCReturn.
        MPCReturn solve(const OptVariables& optVars);
        
        /// @brief solve function for mpc. Uses states and coefficients of a third order polynomial 
        ///         that is threated at desired trajectory.
        /// @param[in] optVars the state of the car. (position, velocity, steering angle, ...)
        /// @param[in] coeffs coefficients of third degree polynomial. Used as desired trajectory.
        /// @return solution from mpc. See definition of MPCReturn.
        MPCReturn solve(const OptVariables& optVars, const Eigen::Vector4d& coeffs);

        /// @brief converts solution from CppAD solver to custom solution class
        /// @param[in] solution solution from CppAD solver. Includes cost, optimal inputs, predicted state and more
        /// @param[in] time time used by solver in [ms]
        /// @return custom solution class for mpc. See MPCReturn for more information.
        MPCReturn toMPCReturn(const CppAD::ipopt::solve_result<Dvector>& solution, double time);
        
        /// @brief model of a car. Used to predict state evolution.
        /// @param[in/out] optVars state thate should be updated
        /// @param[in] u inputs on car (acceleration and steering angle)
        void model(OptVariables& optVars, const Input& u);

    private:

        /// @brief calculates coefficients of third order polynomial that fits the disired trajectory best.
        /// @param[in] state current state of the car.
        /// @param[in] rotation refrence frame roation relative too the car.
        /// @param[out] coeffs the calculated coefficients for the polynomial
        void calcCoeffs(const State& state, double& rotation, Eigen::Vector4d& coeffs) const;

        /// @brief interpolates third order polynomial to fit the track between start and end.
        /// @param[in] state current state. Used for refrence frame rotation and translation.
        /// @param[in] rotation refrence frame rotation relative to car.
        /// @param[in] start the start index for the track section.
        /// @param[in] end the end index for the track section.
        /// @param[out] cost sum of squares between polynomial and points. Least squares method.
        /// @return coefficient for the polynomial found using least squares method.
        Eigen::Vector4d interpolate(const State& state, double rotation, size_t start, size_t end, double& cost) const;
        
        /// @brief calculate the rest of the state. i.e. crosstrack error and yaw error
        /// @param[in/out] state state with valid yaw, x, y and vel. Updated with cte and epsi.
        /// @param[in] coeffs coefficients for third degree polynomial that represents desired trajectory.
        void calcState(State& state, const Eigen::Vector4d& coeffs) const;

        /// @brief calculate y value of third order polynomial. y = f(x)
        /// @param[in] x the x value for the function.
        /// @param[in] coeffs the coeffiecients of the polynomial.
        /// @return y value of fucntion y = f(x)
        double polyEval(double x, const Eigen::Vector4d& coeffs) const {
            return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x; 
        }

        /// @brief calculates track section to be used for interpolation.
        /// @param[out] start the start index for track section.
        /// @param[out] stop end index for track section.
        /// @param[in] state current state of the car.
        void getTrackSection(size_t& start, size_t& stop, const State& state) const;
        
        /// @brief discrete points representing desired trajectory
        std::vector<Point> track_;

        /// @brief publisher for the desired trajectory
        ros::Publisher trackPub_;

        /// @brief publisher for the mpc trajectory solution
        ros::Publisher mpcPathPub_;

        /// @brief publisher for the interpolated polynomial.
        ros::Publisher polynomPub_;

        /// @brief number of steps in mpc solver
        size_t N;

        /// @brief time increment for model.
        double dt;

        /// @brief distance between front and rear wheels
        constexpr static double Lf = MPC_WHEELBASE;

        /// @brief start index for x variables in mpc variables
        const size_t x_start;

        /// @brief start index for y variables in mpc variables
        const size_t y_start;

        /// @brief start index for psi variables in mpc variables
        const size_t psi_start;

        /// @brief start index for vel variables in mpc variables
        const size_t v_start;

        /// @brief start index for cte variables in mpc variables
        const size_t cte_start;

        /// @brief start index for epsi variables in mpc variables
        const size_t epsi_start;

        /// @brief start index for delta variables in mpc variables
        const size_t delta_start;

        /// @brief start index for accel variables in mpc variables
        const size_t a_start;
    };

}
