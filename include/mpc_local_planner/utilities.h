#ifndef MPC_UTILITIES_H_
#define MPC_UTILITIES_H_

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mpc_local_planner/BezierCurve.h"
#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/bounds.h"

// TODO: make more macros for logging
/// @brief node used for logging
extern rclcpp::Node::SharedPtr logNode;
#define LOG_STREAM(...) RCLCPP_INFO_STREAM(logNode->get_logger(), __VA_ARGS__)
#ifdef NDEBUG                                // true if the build type is not debug
#define LOG_DEBUG_STREAM(...) ((void)0)  // define as do nothing
#define LOG_DEBUG(...) ((void)0)         // define as do nothing
#else
#define LOG_DEBUG_STREAM(...) RCLCPP_INFO_STREAM(logNode->get_logger(), __VA_ARGS__)  // print args to console
#define LOG_DEBUG(...) RCLCPP_INFO(logNode->get_logger(), __VA_ARGS__)                // print args to console
#endif

namespace mpc {
namespace util {

/// @brief initialises the logNode
void initLogger();

/// @brief convert State class to Pose message
geometry_msgs::msg::Pose toMsg(const State &state);

/// @brief interpolation for points.
/// @param[in] xvals the x values of the points
/// @param[in] yvals the y values of the points
/// @param[in] order the wanted order of the interpolated polynomial.
/// @exception order must be less than the number of points - 1. order < (xVals.size() -1) and
/// larger than 0
/// @exception xVals must be the same size as yVals. Since they correspond to n points.
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

/// @brief calculate the distance squared
/// @param[in] dx delta x. Difference in the x-axis
/// @param[in] dy delta y. Difference in the y-axis
/// @return the distance of the hypotenuse squared
inline double distSqrd(double dx, double dy) { return dx * dx + dy * dy; }

/// @brief calculate the distance squared between two points.
/// @param[in] p1 start point
/// @param[in] p2 end point
/// @return the distance squared
inline double distSqrd(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
           (p1.z - p2.z) * (p1.z - p2.z);
}

/// @brief get the testTrack for the mpc.
/// @return vector containg the points that define the track.
std::vector<Point> getTestTrack();

/// @brief convert mpcHorizon in solutio to path message
/// @param[in] solution the soluion from mpc. Containg optimal varibles over time.
/// @param[in] mapFrame the frame where the positions are relative to. Often "map", "world" or
/// "odom"
/// @param[in] carFrame the frame of the points. Here the frame of the car.
/// @return Path message from mpcHorizon
nav_msgs::msg::Path getPathMsg(const MPCReturn &solution, const std::string &mapFrame,
                          const std::string &carFrame, rclcpp::Node& node);

/// @brief convert third order polynomial to path message
/// @param[in] coeffs coefficents of the polynomial
/// @param[in] mapFrame the frame where the positions are relative to. Often "map", "world" or
/// "odom"
/// @param[in] carFrame the frame of the points. Here the frame of the car.
/// @return path message with points that lay on the polynomial
nav_msgs::msg::Path getPathMsg(const Eigen::Vector4d &coeffs, const std::string &mapFrame,
                          const std::string &carFrame, rclcpp::Node& node);

/// @brief convert track to path message
/// @param[in] track the track that should be converted
/// @param[in] mapFrame the frame where the positions are relative to. Often "map", "world" or
/// "odom"
/// @param[in] carFrame the frame of the points. Here the frame of the car.
/// @return the resulting path message
nav_msgs::msg::Path getPathMsg(const std::vector<Point> &track, const std::string &mapFrame,
                          const std::string &carFrame, rclcpp::Node& node);

/// @brief convert odometry message to State
/// @param[in] odom odometry message
/// @return State class with the values from odometry message
State toState(const nav_msgs::msg::Odometry &odom);

/// @brief get linear velocity from odometry message
/// @param[in] odom odometry message
/// @return the velocity
double velocity(const nav_msgs::msg::Odometry &odom);

/// @brief get length of 3D vector
/// @param[in] vec 3D vector
/// @return length of vector
double length(const geometry_msgs::msg::Vector3 &vec);

/// @brief get yaw from quaternion
/// @param[in] quat Quaternion
/// @return yaw from quaternion
double getYaw(const geometry_msgs::msg::Quaternion &quat);

/// @brief get pitch from quaternion
/// @param[in] quat Quaternion
/// @return pitch from quaternion
double getPitch(const geometry_msgs::msg::Quaternion &quat);

/// @brief get roll, pitch and yaw from quaternion
/// @param[in] quat Quaternion
/// @param[out] roll roll
/// @param[out] pitch pitch
/// @param[out] yaw yaw
void getRPY(const geometry_msgs::msg::Quaternion &quat, double &roll, double &pitch, double &yaw);

/// @brief get parameter from nh and throw error if the parameter does not exist
/// @param[in] nh the nodhandle containing the parameters
/// @param[in] param the parameter you want to get
/// @return the parameter as the template type
template<typename T>
T getParamError(const rclcpp::Node& node, const std::string &param) {
    rclcpp::Parameter ros_param;
    if (!node.get_parameter(param, ros_param)) {
        std::string err = "Could not find parameter: " + param + ". This should be specified when launchig.";
        RCLCPP_ERROR_STREAM(node.get_logger(), err);
        throw std::runtime_error{err};
    }
    return ros_param.get_parameter_value().get<T>();
}

/// @brief get parameter from nh and print warning if the parameter does not exist
/// @param[in] nh the nodhandle containing the parameters
/// @param[in] param the parameter you want to get
/// @return the parameter as the template type
template<typename T>
T getParamWarn(const rclcpp::Node& node, const std::string &param, const T& default_value) {
    rclcpp::Parameter ros_param;
    if (!node.get_parameter_or(param, ros_param, rclcpp::Parameter{param, default_value})) {
        std::string err = "Could not find parameter: " + param + ". This should be specified when launchig.";
        RCLCPP_WARN_STREAM(node.get_logger(), err);
    }
    return ros_param.get_parameter_value().get<T>();
}

/// @brief converts path message to vector of points.
/// @param[in] path the path message we want to convert.
/// @return the points stored in a vector.
std::vector<Point> toVector(const nav_msgs::msg::Path &path);

/// @brief convert 3D point message to 2D Point. Removing the z value.
/// @param[in] p the point message.
/// @return mpc::Point object with the x and y values of p.
Point toPoint(const geometry_msgs::msg::Point &p);

/// @brief get path message from a bezier curve
/// @param[in] curve the curve we want to convert to a path message
/// @param[in] mapFrame global frame of path message
/// @param[in] carFrame the frame of the car
/// @return path message
nav_msgs::msg::Path getPathMsg(const mpc::BezierCurve &curve, const std::string &mapFrame,
                          const std::string &carFrame, rclcpp::Node& node);

/// @brief convert bezier curve to vector with points
/// @param[in] curve the bezier curve
/// @return vector with points on the bezier curve
std::vector<Point> getPath(const mpc::BezierCurve &curve);

}  // namespace util
}  // namespace mpc
#endif