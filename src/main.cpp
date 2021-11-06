#include "mpc_local_planner/utilities.h"
#include "mpc_local_planner/MPC.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");
    auto track = mpc::getTestTrack();
    mpc::State initialState{23, 0, M_PI_2, 0, 0, 0};
    constexpr size_t N = 30;
    constexpr double dt = 0.1;

    mpc::State state = initialState;
    mpc::MPC MPC{track, N, dt};
    std::cout << "x\ty\tpsi\tvel\tcost\n";
    auto refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{100};
    std::this_thread::sleep_for(std::chrono::seconds(2)); // wait for rviz
    while(ros::ok()){
        auto solution = MPC.solve(state);
        mpc::logSolution(solution, state, "log.txt");
        MPC.model(state, solution.u0);
        std::cout << state.x << '\t' << state.y << '\t' << state.psi 
            << '\t' << state.vel << '\t' << solution.cost << std::endl;
        std::cout << "Time: " << solution.computeTime << " [ms]\n";
        if (solution.cost < 100 && solution.cost > 0.1){
            state = initialState;
        }
        std::this_thread::sleep_until(refresh);
        refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{(int)(1000 * dt)};
    }
    return 0;
}