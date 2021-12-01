#pragma once

#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <cppad/cppad.hpp>

namespace mpc{

    /// @brief convert State class to Pose message
    geometry_msgs::Pose toMsg(const State& state);
    
    /// @brief convert a std::vector too a CppAD vector
    CPPAD_TESTVECTOR(double) toCppAD(const std::vector<double>& vec);

    /// @brief save solution to file
    /// @param[in] solution the solution that should be solved
    /// @param[in] curState the current state of the car
    /// @param[in] filename the filename inside the log folder.
    void logSolution(const MPCReturn& solution, const State& curState, const std::string& filename);

    /// @brief interpolation for points.
    /// @param[in] xvals the x values of the points
    /// @param[in] yvals the y values of the points
    /// @param[in] order the wanted order of the interpolated polynomial. 
    /// @exception order must be less than the number of points - 1. order < (xVals.size() -1) and larger than 0
    /// @exception xVals must be the same size as yVals. Since they correspond to n points.
    Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

    /// @brief calculate the distance squared
    inline double distSqrd(double dx, double dy) {
        return dx * dx + dy * dy;
    }

    /// @brief get the testTrack for the mpc.
    /// @return vector containg the points that define the track.
    std::vector<Point> getTestTrack();

    /// @brief convert mpcHorizon in solutio to path message
    /// @param[in] solution the soluion from mpc. Containg optimal varibles over time.
    /// @return Path message from mpcHorizon
    nav_msgs::Path getPathMsg(const MPCReturn& solution);

    /// @brief convert third order polynomial to path message
    /// @param[in] coeffs coefficents of the polynomial
    /// @return path message with points that lay on the polynomial
    nav_msgs::Path getPathMsg(const Eigen::Vector4d& coeffs);

    /// @brief convert track to path message
    /// @param[in] track the track that should be converted
    /// @return the resulting path message
    nav_msgs::Path getPathMsg(const std::vector<Point>& track);

    /// @brief convert odometry message to State
    /// @param[in] odom odometry message
    /// @return State class with the values from odometry message
    State toState(const nav_msgs::Odometry& odom);

    /// @brief get linear velocity from odometry message
    /// @param[in] odom odometry message
    /// @return the velocity
    double velocity(const nav_msgs::Odometry& odom);   

    /// @brief get length of 3D vector
    /// @param[in] vec 3D vector
    /// @return length of vector
    double length(const geometry_msgs::Vector3& vec);

    /// @brief get yaw from quaternion
    /// @param[in] quat Quaternion
    /// @return yaw from quaternion
    double getYaw(const geometry_msgs::Quaternion& quat);
}