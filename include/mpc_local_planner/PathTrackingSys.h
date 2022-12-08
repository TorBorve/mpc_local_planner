#ifndef MPC_MPC_H_
#define MPC_MPC_H_

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include <eigen3/Eigen/Core>

#include "mpc_local_planner/AcadosPathTracking.h"
#include "mpc_local_planner/AcadosPointStab.h"
#include "mpc_local_planner/bounds.h"
#include "mpc_local_planner/types.h"

namespace mpc {

/// @brief path tracking class for car
class PathTrackingSys {
   public:
    /// @brief constructor for path tracking class
    /// @param[in] track vector with points that define desired trajectory
    PathTrackingSys(const std::vector<Point> &track);

    /// @brief update track variable for desired trajectory
    /// @param[in] newTrack the new desired trajectory
    void setTrack(const std::vector<Point> &newTrack) { track_ = newTrack; }

    /// @brief get the current desired trajectory
    /// @return track for the desired trajectory
    std::vector<Point> getTrack() const { return track_; }

    /// @brief solve function for path tracking. Solves the nlp with state as given.
    /// @param[in] state the state of the car.
    /// @param[in] pitch the pitch angle of the car
    /// @return solution from path tracking. See definition of MPCReturn.
    MPCReturn solve(const State &state, double pitch, double vRef);

    /// @brief solve function for path tracking. Uses states and coefficients of a third order
    /// polynomial
    ///        that is threated at desired trajectory.
    /// @param[in] state the state of the car. (position, velocity, steering angle, ...)
    /// @param[in] params parameters for solver.
    /// @return solution from path tracking. See definition of MPCReturn.
    MPCReturn solve(const State &state, const Acados::PathTrackingParams &params);

   private:
    /// @brief calculates coefficients of third order polynomial that fits the disired trajectory
    /// best.
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
    Eigen::Vector4d interpolate(const State &state, double rotation, size_t start, size_t end,
                                double &cost) const;

    /// @brief calculate the rest of the state. i.e. crosstrack error and yaw error
    /// @param[in/out] state state with valid yaw, x, y and vel. Updated with cte and epsi.
    /// @param[in] coeffs coefficients for third degree polynomial that represents desired
    /// trajectory.
    void calcState(State &state, const Eigen::Vector4d &coeffs) const;

    /// @brief calculate y value of third order polynomial. y = f(x)
    /// @param[in] x the x value for the function.
    /// @param[in] coeffs the coeffiecients of the polynomial.
    /// @return y value of fucntion y = f(x)
    double polyEval(double x, const Eigen::Vector4d &coeffs) const {
        return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x;
    }

    /// @brief calculates track section to be used for interpolation.
    /// @param[out] start the start index for track section.
    /// @param[out] stop end index for track section.
    /// @param[in] state current state of the car.
    void getTrackSection(size_t &start, size_t &stop, const State &state) const;

    /// @brief discrete points representing desired trajectory
    std::vector<Point> track_;

    /// @brief publisher for the interpolated polynomial.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr polynomPub_;
};

}  // namespace mpc

#endif