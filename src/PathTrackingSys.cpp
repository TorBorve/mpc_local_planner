#include "mpc_local_planner/PathTrackingSys.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fstream>

#include "mpc_local_planner/AcadosPathTracking.h"
#include "mpc_local_planner/bounds.h"
#include "mpc_local_planner/utilities.h"

namespace mpc {

PathTrackingSys::PathTrackingSys(const std::vector<Point> &track) : track_{track} {
    polynomPub_ = logNode->create_publisher<nav_msgs::msg::Path>("interpolated_path", 1);
}

MPCReturn PathTrackingSys::solve(const State &state, double pitch, double vRef) {
    double rotation = 0;
    Eigen::Vector4d coeffs;
    calcCoeffs(state, rotation, coeffs);
    Acados::PathTrackingParams params{coeffs, pitch, vRef};

    State transformedState{0, 0, rotation, state.vel, state.delta, state.throttle};

    auto result = solve(transformedState, params);

    const double rotAngle = state.psi - rotation;
    const double sinRot = sin(rotAngle);
    const double cosRot = cos(rotAngle);
    auto &x = result.mpcHorizon;
    for (unsigned int i = 0; i < x.size(); i++) {
        // rotate back
        double dx = x[i].x.x;
        double dy = x[i].x.y;
        x[i].x.x = dx * cosRot - dy * sinRot;
        x[i].x.y = dx * sinRot + dy * cosRot;

        // shift coordinates
        x[i].x.x += state.x;
        x[i].x.y += state.y;
    }

    auto polyPath = util::getPathMsg(coeffs, "odom", "base_footprint", *logNode);
    auto &points = polyPath.poses;
    for (unsigned int i = 0; i < points.size(); i++) {
        double dx = points[i].pose.position.x;
        double dy = points[i].pose.position.y;
        points[i].pose.position.x = dx * cosRot - dy * sinRot;
        points[i].pose.position.y = dx * sinRot + dy * cosRot;

        points[i].pose.position.x += state.x;
        points[i].pose.position.y += state.y;
    }
    polynomPub_->publish(polyPath);  // TODO: Ideally this would be in RosMPC.cpp
    return result;
}

MPCReturn PathTrackingSys::solve(const State &state, const Acados::PathTrackingParams &params) {
    static Acados::PathTracking solver{state};
    return solver.solve(state, params);
}

void PathTrackingSys::calcCoeffs(const State &state, double &rotation,
                                 Eigen::Vector4d &coeffs) const {
    size_t start, end;
    getTrackSection(start, end, state);

    double minCost = 1e19;
    for (double rot = -M_PI_2; rot < 0; rot += M_PI_2 / 3) {
        double curCost = 1e19;
        Eigen::Vector4d curCoeffs = interpolate(state, rot, start, end, curCost);
        if (curCost < minCost) {
            minCost = curCost;
            coeffs = curCoeffs;
            rotation = rot;
        }
    }
    return;
}

Eigen::Vector4d PathTrackingSys::interpolate(const State &state, double rotation, size_t start,
                                             size_t end, double &cost) const {
    Eigen::VectorXd xVals(end - start);
    Eigen::VectorXd yVals(end - start);
    double angle = rotation - state.psi;

    for (unsigned int i = start; i < end; i++) {
        // shift points so that the state is in the origo
        double dx = track_[i].x - state.x;
        double dy = track_[i].y - state.y;

        // rotate points so that state.psi = 0 in the new refrence frame
        xVals[i - start] = dx * cos(angle) - dy * sin(angle);
        yVals[i - start] = dx * sin(angle) + dy * cos(angle);
    }

    auto coeffs = util::polyfit(xVals, yVals, 3);
    assert(coeffs.size() == 4);

    cost = 0;
    for (unsigned int i = 0; i < yVals.size(); i++) {
        cost += util::distSqrd(yVals[i] - polyEval(xVals[i], coeffs), 0);
    }
    return coeffs;
}

void PathTrackingSys::getTrackSection(size_t &start, size_t &end, const State &state) const {
    double maxLen = 15;
    double minDistSqrd = util::distSqrd(state.x - track_[0].x, state.y - track_[0].y);
    size_t minIndex = 0;
    for (unsigned int i = 1; i < track_.size(); i++) {
        double curDistSqrd = util::distSqrd(state.x - track_[i].x, state.y - track_[i].y);
        if (curDistSqrd < minDistSqrd) {
            minDistSqrd = curDistSqrd;
            minIndex = i;
        }
    }

    double len = 0;
    size_t frontIndex = minIndex;
    while (len < maxLen && frontIndex < track_.size() - 1) {
        frontIndex++;
        len += sqrt(util::distSqrd(track_[frontIndex].x - track_[frontIndex - 1].x,
                                   track_[frontIndex].y - track_[frontIndex - 1].y));
    }
    start = minIndex;
    end = frontIndex;
    if (end - start < 4) {
        end = start + 4;
    }
    if (end >= track_.size()) {
        start = 0;
        end = start + 4;
    }
    assert(end < track_.size());
}
}  // namespace mpc