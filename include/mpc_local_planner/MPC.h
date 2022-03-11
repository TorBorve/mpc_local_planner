#ifndef MPC_MPC_H_
#define MPC_MPC_H_

#include <eigen3/Eigen/Core>
#include <ros/ros.h>

#include "mpc_local_planner/types.h"
#include "mpc_local_planner/bounds.h"

namespace mpc
{

    /// @brief MPC class for car
    class MPC
    {
    public:
        /// @brief constructor for MPC class
        /// @param[in] track vector with points that define desired trajectory
        /// @param[in] N number of steps in simulation
        /// @param[in] dt time increment for model
        /// @param[in] steeringAngle bounds for max and min steering angle
        /// @param[in] maxSteeringRotationsSpeed the maximum speed the wheels can turn at. Given in [rad/s].
        /// @param[in] wheelbase the distance between the front and rear wheels.
        MPC(const std::vector<Point> &track, size_t N, double dt, Bound steeringAngle, double maxSteeringRotationSpeed, double wheelbase);

        /// @brief update track variable for desired trajectory
        /// @param[in] newTrack the new desired trajectory
        void setTrack(const std::vector<Point> &newTrack)
        {
            track_ = newTrack;
        }

        /// @brief get the current desired trajectory
        /// @return track for the desired trajectory
        std::vector<Point> getTrack() const
        {
            return track_;
        }

        /// @brief solve function for mpc. Solves the nlp with state as given.
        /// @param[in] optVars the state of the car. (position, velocity, steering angle, ...)
        /// @return solution from mpc. See definition of MPCReturn.
        MPCReturn solve(const OptVariables &optVars);

        /// @brief solve function for mpc. Uses states and coefficients of a third order polynomial
        ///        that is threated at desired trajectory.
        /// @param[in] optVars the state of the car. (position, velocity, steering angle, ...)
        /// @param[in] coeffs coefficients of third degree polynomial. Used as desired trajectory.
        /// @return solution from mpc. See definition of MPCReturn.
        MPCReturn solve(const OptVariables &optVars, const Eigen::Vector4d &coeffs);

        /// @brief model of a car. Used to predict state evolution.
        /// @param[in/out] optVars state thate should be updated
        /// @param[in] u inputs on car (acceleration and steering angle)
        void model(OptVariables &optVars, const Input &u);

        /// @brief model of a car. Used to predict state evolution.
        /// @param[in/out] optVars state thate should be updated
        /// @param[in] u inputs on car (acceleration and steering angle)
        /// @param[in] dt the time increment used in the model
        void model(OptVariables &optVars, const Input &u, double dt);

    private:
        /// @brief calculates coefficients of third order polynomial that fits the disired trajectory best.
        /// @param[in] state current state of the car.
        /// @param[in] rotation refrence frame roation relative too the car.
        /// @param[out] coeffs the calculated coefficients for the polynomial
        void calcCoeffs(const State &state, double &rotation, Eigen::Vector4d &coeffs) const;

        /// @brief interpolates third order polynomial to fit the track between start and end.
        /// @param[in] state current state. Used for refrence frame rotation and translation.
        /// @param[in] rotation refrence frame rotation relative to car.
        /// @param[in] start the start index for the track section.
        /// @param[in] end the end index for the track section.
        /// @param[out] cost sum of squares between polynomial and points. Least squares method.
        /// @return coefficient for the polynomial found using least squares method.
        Eigen::Vector4d interpolate(const State &state, double rotation, size_t start, size_t end, double &cost) const;

        /// @brief calculate the rest of the state. i.e. crosstrack error and yaw error
        /// @param[in/out] state state with valid yaw, x, y and vel. Updated with cte and epsi.
        /// @param[in] coeffs coefficients for third degree polynomial that represents desired trajectory.
        void calcState(State &state, const Eigen::Vector4d &coeffs) const;

        /// @brief calculate y value of third order polynomial. y = f(x)
        /// @param[in] x the x value for the function.
        /// @param[in] coeffs the coeffiecients of the polynomial.
        /// @return y value of fucntion y = f(x)
        double polyEval(double x, const Eigen::Vector4d &coeffs) const
        {
            return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x;
        }

        /// @brief calculates track section to be used for interpolation.
        /// @param[out] start the start index for track section.
        /// @param[out] stop end index for track section.
        /// @param[in] state current state of the car.
        void getTrackSection(size_t &start, size_t &stop, const State &state) const;

        /// @brief publishes transform from map to car. This is used to verify that the position that
        /// the mpc recives is correct
        /// @param[in] state the state give to the mpc
        void pubTf(const State &state) const;

        /// @brief discrete points representing desired trajectory
        std::vector<Point> track_;

        /// @brief publisher for the desired trajectory
        ros::Publisher trackPub_;

        /// @brief publisher for the mpc trajectory solution
        ros::Publisher mpcPathPub_;

        /// @brief publisher for the interpolated polynomial.
        ros::Publisher polynomPub_;

        /// @brief number of steps in mpc solver
        size_t N_;

        /// @brief time increment for model.
        double dt_;

        /// @brief distance between front and rear wheels
        double wheelbase_;

        /// @brief bounds for max and min steering angle
        Bound steeringAngle_;

        /// @brief maxSteeringRotationsSpeed the maximum speed the wheels can turn at. Given in [rad/s].
        double maxSteeringRotationSpeed_;
    };

}

#endif