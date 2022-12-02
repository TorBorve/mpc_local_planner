#include "mpc_local_planner/ParkingSys.h"

#include "mpc_local_planner/utilities.h"

namespace mpc {

MPCReturn ParkingSys::solve(const State &state, double pitch) {
    if (!init_ || mode_ == Mode::Invalid) {
        return MPCReturn{};
    }
    double distSqrdToGoal = util::distSqrd(state.x - goal_.position.x, state.y - goal_.position.y);
    if (distSqrdToGoal < 5 * 5) {
        Acados::PointStabParams params;
        params.pitch = pitch;
        params.pRef = Point{goal_.position.x, goal_.position.y};
        params.psiRef = util::getYaw(goal_.orientation);
        auto res = pointStabSolver_.solve(state, params);
        if (distSqrdToGoal < 2 * 2 && mode_ == Mode::Parking) {
            res.stopSignal = true;
        }
        return res;
    } else {
        if (updateStart_) {
            startState_ = state;
            updateStart_ = false;
        }
        if (updatePath_) {
            createPathToGoal();
            updatePath_ = false;
        }
        return pathTrackingSys_.solve(state, pitch, refVel_);
    }
}

void ParkingSys::createPathToGoal() {
    Point car{startState_.x, startState_.y};
    Point goal{goal_.position.x, goal_.position.y};
    double frac = 0;
    if (mode_ == Mode::Parking) {
        frac = 0.5;
    } else if (mode_ == Mode::Slalom) {
        frac = 0.25;
    }
    BezierCurve bCurve{car, startState_.psi, goal, util::getYaw(goal_.orientation), frac};
    pathTrackingSys_.setTrack(util::getPath(bCurve));
}

void ParkingSys::setRefPose(const geometry_msgs::msg::Pose &pose) {
    if (!init_) {
        goal_ = pose;
        init_ = true;
        updatePath_ = true;
        updateStart_ = true;
    }
    double dist2 = util::distSqrd(pose.position, goal_.position);
    double diffYaw = abs(util::getYaw(pose.orientation) - util::getYaw(goal_.orientation));
    if (diffYaw > M_PI) {
        diffYaw = 2 * M_PI - diffYaw;
    }

    if (dist2 > 0.3 * 0.3 || diffYaw > 15 * M_PI / 180.0) {
        goal_ = pose;
        updatePath_ = true;
        if (dist2 > 2 * 2 || diffYaw > 45 * M_PI / 180.0) {
            updateStart_ = true;
        }
    }
}

}  // namespace mpc