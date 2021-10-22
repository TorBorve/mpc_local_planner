#include "mpc_local_planner/CppADMPC.h"
#include "mpc_local_planner/utilities.h"
#include "mpc_local_planner/TrajectoryTracking/TrajectoryMPC.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");
    ros::NodeHandle nh;
    ros::Publisher pathPub;
    pathPub = nh.advertise<nav_msgs::Path>("local_path", 1);
    // mpc::State refState{10, 10, M_PI};
    Eigen::Vector4d coeffs;
    coeffs << 0, 0, 0, -0.01;
    mpc::State state{0, 2, M_PI, 0};
    mpc::nonlinear::TrajectoryTracking::MPC mpc{coeffs};
    std::cout << "x\ty\tpsi\tvel\tcost\n";
    auto refresh = std::chrono::high_resolution_clock::now();
    while(true){
        auto fullState = mpc.calcState(state);
        // std::cout << fullState[4] << '\t' << fullState[5] << '\n';
        auto solution = mpc.solve(fullState);
        state = solution.mpcHorizon[1].x;
        std::cout << state.x << '\t' << state.y << '\t' << state.psi 
            << '\t' << state.vel << '\t' << solution.cost << std::endl;
        // std::cout << "Time: " << solution.computeTime << " [ms]\n";
        pathPub.publish(getPathMsg(solution));
        if (solution.cost < 100){
            state = mpc::State{0, 0, 0, 0};
        }
        refresh += std::chrono::milliseconds{100};
        std::this_thread::sleep_until(refresh);
    }

    return 0;
}