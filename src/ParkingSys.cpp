#include "mpc_local_planner/ParkingSys.h"

#include "mpc_local_planner/utilities.h"

namespace mpc {

MPCReturn ParkingSys::solve(const State &state, double pitch) {
    if (!init_) {
        return MPCReturn{};
    }
    double distSqrdToGoal = distSqrd(state.x - goal_.position.x, state.y - goal_.position.y);
    if (distSqrdToGoal < 5 * 5) {
        Acados::PointStabParams params;
        params.pitch = pitch;
        params.pRef = Point{goal_.position.x, goal_.position.y};
        params.psiRef = getYaw(goal_.orientation);
        return pointStabSolver_.solve(state, params);

    } else {
        m.lock();
        if (updateStart_) {
            startState_ = state;
            updateStart_ = false;
        }
        if (updatePath_) {
            createPathToGoal();
            updatePath_ = false;
        }
        m.unlock();
        return pathTrackingSys_.solve(state, pitch, 1.0);
    }
}

void ParkingSys::createPathToGoal() {
    Point car{startState_.x, startState_.y};
    Point goal{goal_.position.x, goal_.position.y};
    BezierCurve bCurve{car, startState_.psi, goal, getYaw(goal_.orientation)};
    pathTrackingSys_.setTrack(getPath(bCurve));
}

void ParkingSys::setRefPose(const geometry_msgs::Pose &pose) {
    if (!init_) {
        goal_ = pose;
        init_ = true;
        m.lock();
        updatePath_ = true;
        updateStart_ = true;
        m.unlock();
    }
    double dist2 = distSqrd(pose.position, goal_.position);
    double diffYaw = abs(getYaw(pose.orientation) - getYaw(goal_.orientation));
    if (diffYaw > M_PI) {
        diffYaw = 2 * M_PI - diffYaw;
    }
    
    if (dist2 > 0.3*0.3 || diffYaw > 15*M_PI/180.0) {
        goal_ = pose;
        m.lock();
        updatePath_ = true;
        if (dist2 > 2*2 || diffYaw > 45*M_PI/180.0) {
            updateStart_ = true;
        }
        m.unlock();
    }
}

}  // namespace mpc