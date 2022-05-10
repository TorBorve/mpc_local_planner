#ifndef MPC_UTILITIES_H_
#define MPC_UTILITIES_H_

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "mpc_local_planner/PathTrackingSys.h"
#include "mpc_local_planner/bounds.h"

#ifdef NDEBUG                                // true if the build type is not debug
#define LOG_DEBUG_STREAM(args...) ((void)0)  // define as do nothing
#define LOG_DEBUG(args...) ((void)0)         // define as do nothing
#else
#define LOG_DEBUG_STREAM(args...) \
    ROS_INFO_STREAM(args)  // print args to consol
#define LOG_DEBUG(args...) \
    ROS_INFO(args)  // print args to consol
#endif

namespace mpc {

/// @brief convert State class to Pose message
geometry_msgs::Pose toMsg(const State &state);

/// @brief interpolation for points.
/// @param[in] xvals the x values of the points
/// @param[in] yvals the y values of the points
/// @param[in] order the wanted order of the interpolated polynomial.
/// @exception order must be less than the number of points - 1. order < (xVals.size() -1) and larger than 0
/// @exception xVals must be the same size as yVals. Since they correspond to n points.
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

/// @brief calculate the distance squared
inline double distSqrd(double dx, double dy) {
    return dx * dx + dy * dy;
}

/// @brief get the testTrack for the mpc.
/// @return vector containg the points that define the track.
std::vector<Point> getTestTrack();

/// @brief convert mpcHorizon in solutio to path message
/// @param[in] solution the soluion from mpc. Containg optimal varibles over time.
/// @param[in] mapFrame the frame where the positions are relative to. Often "map", "world" or "odom"
/// @param[in] carFrame the frame of the points. Here the frame of the car.
/// @return Path message from mpcHorizon
nav_msgs::Path getPathMsg(const MPCReturn &solution, const std::string &mapFrame, const std::string &carFrame);

/// @brief convert third order polynomial to path message
/// @param[in] coeffs coefficents of the polynomial
/// @param[in] mapFrame the frame where the positions are relative to. Often "map", "world" or "odom"
/// @param[in] carFrame the frame of the points. Here the frame of the car.
/// @return path message with points that lay on the polynomial
nav_msgs::Path getPathMsg(const Eigen::Vector4d &coeffs, const std::string &mapFrame, const std::string &carFrame);

/// @brief convert track to path message
/// @param[in] track the track that should be converted
/// @param[in] mapFrame the frame where the positions are relative to. Often "map", "world" or "odom"
/// @param[in] carFrame the frame of the points. Here the frame of the car.
/// @return the resulting path message
nav_msgs::Path getPathMsg(const std::vector<Point> &track, const std::string &mapFrame, const std::string &carFrame);

/// @brief convert odometry message to State
/// @param[in] odom odometry message
/// @return State class with the values from odometry message
State toState(const nav_msgs::Odometry &odom);

/// @brief get linear velocity from odometry message
/// @param[in] odom odometry message
/// @return the velocity
double velocity(const nav_msgs::Odometry &odom);

/// @brief get length of 3D vector
/// @param[in] vec 3D vector
/// @return length of vector
double length(const geometry_msgs::Vector3 &vec);

/// @brief get yaw from quaternion
/// @param[in] quat Quaternion
/// @return yaw from quaternion
double getYaw(const geometry_msgs::Quaternion &quat);

/// @brief get pitch from quaternion
/// @param[in] quat Quaternion
/// @return pitch from quaternion
double getPitch(const geometry_msgs::Quaternion &quat);

/// @brief get roll, pitch and yaw from quaternion
/// @param[in] quat Quaternion
/// @param[out] roll roll
/// @param[out] pitch pitch
/// @param[out] yaw yaw
void getRPY(const geometry_msgs::Quaternion &quat, double &roll, double &pitch, double &yaw);

/// @brief checks if param exist in nh. If not an error is printed and a exception is thrown
/// @param[in] nh pointer to nodeHandle that should have access to the parameter.
/// @param[in] param the parameter we want to check if exists.
/// @return true if the parameter existed
/// @throw runtime error it the parameter didn't exist
bool hasParamError(ros::NodeHandle *nh, const std::string &param);

/// @brief checks if the parameter exist in nh. If not an warning is printed.
/// @param[in] nh pointer to nodeHandle that should have access to the parameter.
/// @param[in] param the parameter we want to check if exists.
/// @return true if the parameter existed
bool hasParamWarn(ros::NodeHandle *nh, const std::string &param);

/// @brief converts path message to vector of points.
/// @param[in] path the path message we want to convert.
/// @return the points stored in a vector.
std::vector<Point> toVector(const nav_msgs::Path &path);

/// @brief convert 3D point message to 2D Point. Removing the z value.
/// @param[in] p the point message.
/// @return mpc::Point object with the x and y values of p.
Point toPoint(const geometry_msgs::Point &p);

class BezierCurve {
   public:
    BezierCurve(const Point &p1, double angle1, const Point &p2, double angle2) {
        double dist = sqrt(distSqrd(p1.x - p2.x, p1.y - p2.y));
        constexpr double frac = 1 / 2.0;
        Point p1Tangent{p1.x + frac * cos(angle1) * dist, p1.y + frac *sin(angle1) * dist};
        Point p2Tangent{p2.x - frac * cos(angle2) * dist, p2.y - frac * sin(angle2) * dist};
        *this = BezierCurve{p1, p1Tangent, p2Tangent, p2};
    }
    BezierCurve(std::initializer_list<Point> list) : points{list} {}
    static Point calc(const std::vector<Point> &points, double t) {
        std::vector<Point> pCopy = points;
        for (int n = points.size(); n > 0; n--) {
            for (int i = 0; i < n - 1; i++) {
                pCopy[i] = lerp(pCopy[i], pCopy[i + 1], t);
            }
        }
        return pCopy[0];
    }
    Point calc(double t) const {
        return calc(points, t);
    }
    static Point lerp(const Point &p1, const Point &p2, double t) {
        double dx = (p2.x - p1.x) * t;
        double dy = (p2.y - p1.y) * t;
        return Point{p1.x + dx, p1.y + dy};
    }
    std::vector<Point> points;
};

nav_msgs::Path getPathMsg(const BezierCurve &curve, const std::string &mapFrame, const std::string &carFrame);

std::vector<Point> getPath(const BezierCurve &curve);
}  // namespace mpc

#endif