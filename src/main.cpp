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
    mpc::State initialState{41, 0, M_PI_2, 0, 0, 0};
    constexpr size_t N = 30;
    constexpr double dt = 0.1;

    mpc::State state = initialState;
    mpc::MPC MPC{track, N, dt};
    std::cout << "x\ty\tpsi\tvel\tcost\n";
    auto refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{100};
    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for rviz
    double totalTime = 0;
    size_t iterations = 0;
    while(ros::ok()){
        auto solution = MPC.solve(state);
        
        std::string filePath = __PATH__;
        filePath += "/logs/log.txt";
        mpc::logSolution(solution, state, filePath);
        MPC.model(state, solution.u0);

        std::cout << "Time: " << solution.computeTime << " [ms]\n";
        totalTime += solution.computeTime;
        iterations++;
        std::cout << "Avg. time: " << totalTime / iterations << " [ms]\n";

        std::this_thread::sleep_until(refresh);
        refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{(int)(1000 * dt)};
    }
    return 0;
}