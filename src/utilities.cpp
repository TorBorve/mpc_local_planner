#include "mpc_local_planner/utilities.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>

namespace mpc {
namespace util {
geometry_msgs::Pose toMsg(const State &state) {
    geometry_msgs::Pose pose;
    pose.position.x = state.x;
    pose.position.y = state.y;
    tf2::Quaternion quat;
    quat.setEuler(state.psi, 0, 0);
    pose.orientation.x = quat.getX();
    pose.orientation.y = quat.getY();
    pose.orientation.z = quat.getZ();
    pose.orientation.w = quat.getW();
    return pose;
}

// Fit a polynomial.
// Adapted from
// https://gist.github.com/ksjgh/4d5050d0e9afc5fdb5908734335138d0
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

std::vector<Point> getTestTrack() {
    constexpr size_t n = 400;
    std::vector<Point> track;
    // double x_start = -30, x_stop = 30;
    // for(double x = x_start; x < x_stop; x += (x_stop - x_start) / n) {
    //     track.push_back(Point{x, 0.5 * x + 0.1 * x * x + -0.01 * x * x * x});
    // }
    for (double theta = 0; theta < 2 * M_PI; theta += 2 * M_PI / (double)n) {
        double radius = 20;
        track.push_back(Point{2 * radius * cos(theta), radius * sin(1 * theta) + radius / 1.2 * sin(3 * theta)});
    }
    return track;
}

nav_msgs::Path getPathMsg(const MPCReturn &solution, const std::string &mapFrame, const std::string &carFrame) {
    nav_msgs::Path path;
    std_msgs::Header header;
    header.frame_id = mapFrame;
    header.stamp = ros::Time::now();
    path.header = header;
    header.frame_id = carFrame;
    path.poses.resize(solution.mpcHorizon.size());
    for (int i = 0; i < path.poses.size(); i++) {
        path.poses[i].pose = toMsg(solution.mpcHorizon[i].x);
        path.poses[i].header = header;
    }
    return path;
}

nav_msgs::Path getPathMsg(const Eigen::Vector4d &coeffs, const std::string &mapFrame, const std::string &carFrame) {
    double start = -30, finish = 30;
    double step = 0.5;
    nav_msgs::Path path;
    std_msgs::Header header;
    header.frame_id = mapFrame;
    header.stamp = ros::Time::now();
    path.header = header;
    header.frame_id = carFrame;
    for (double x = start; x < finish; x += step) {
        double y = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.header = header;
        path.poses.push_back(pose);
    }
    return path;
}

nav_msgs::Path getPathMsg(const std::vector<Point> &track, const std::string &mapFrame, const std::string &carFrame) {
    nav_msgs::Path path;
    std_msgs::Header header;
    header.frame_id = mapFrame;
    header.stamp = ros::Time::now();
    path.header = header;
    header.frame_id = carFrame;
    for (const auto &p : track) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.header = header;
        path.poses.push_back(pose);
    }
    return path;
}

State toState(const nav_msgs::Odometry &odom) {
    double vel = velocity(odom);
    double psi = getYaw(odom.pose.pose.orientation);
    return State{odom.pose.pose.position.x, odom.pose.pose.position.y, psi, vel, 0, 0};
}

double velocity(const nav_msgs::Odometry &odom) {
    return length(odom.twist.twist.linear);
}

double length(const geometry_msgs::Vector3 &vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

double getYaw(const geometry_msgs::Quaternion &quat) {
    double roll, pitch, yaw;
    getRPY(quat, roll, pitch, yaw);
    return yaw;
}

double getPitch(const geometry_msgs::Quaternion &quat) {
    double roll, pitch, yaw;
    getRPY(quat, roll, pitch, yaw);
    return pitch;
}

void getRPY(const geometry_msgs::Quaternion &quat, double &roll, double &pitch, double &yaw) {
    tf2::Quaternion q{quat.x, quat.y, quat.z, quat.w};
    tf2::Matrix3x3 mat{q};
    mat.getRPY(roll, pitch, yaw);
}

bool hasParamError(ros::NodeHandle *nh, const std::string &param) {
    if (!nh->hasParam(param)) {
        std::string error = "parameter " + param + " is not set. This needs to be specified as a rosparameter in the launchfile";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error{error};
    }
    return true;
}

bool hasParamWarn(ros::NodeHandle *nh, const std::string &param) {
    if (!nh->hasParam(param)) {
        std::string error = "parameter " + param + " is not set. This needs to be specified as a rosparameter in the launchfile";
        ROS_WARN_STREAM(error);
        return false;
    }
    return true;
}

std::vector<Point> toVector(const nav_msgs::Path &path) {
    std::vector<Point> pathVector(path.poses.size());
    for (unsigned int i = 0; i < pathVector.size(); i++) {
        pathVector.at(i) = toPoint(path.poses.at(i).pose.position);
    }
    return pathVector;
}

Point toPoint(const geometry_msgs::Point &p) {
    return Point{p.x, p.y};
}

nav_msgs::Path getPathMsg(const BezierCurve &curve, const std::string &mapFrame, const std::string &carFrame) {
    return getPathMsg(getPath(curve), mapFrame, carFrame);
}

std::vector<Point> getPath(const BezierCurve &curve) {
    constexpr size_t n = 100;
    std::vector<Point> path;
    for (double t = 0; t <= 1; t += 1.0 / n) {
        Point p = curve.calc(t);
        path.push_back(p);
    }
    return path;
}
}  // namespace util
}  // namespace mpc