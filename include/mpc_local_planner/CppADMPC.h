#pragma once

#include "eigen3/Eigen/Core"


class MPC {
public:
    MPC(const Eigen::Vector3d& refstate) : refState{refstate}
    {

    }
    void setRefState(const Eigen::Vector3d& newRefState){
        refState = newRefState;
    }
    Eigen::Vector3d getRefState() const {
        return refState;
    }
    void solve(const Eigen::Vector3d& state);
private:
    Eigen::Vector3d refState;
};