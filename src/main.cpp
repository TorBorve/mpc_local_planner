#include "mpc_local_planner/utilities.h"
#include "mpc_local_planner/MPC.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <ros/ros.h>

nav_msgs::Path getPathFromPoly(const Eigen::Vector4d& coeffs){
    double start = -30, finish = 30;
    double step = 0.1;
    nav_msgs::Path path;
    std_msgs::Header header;
    header.frame_id = "odom";
    header.stamp = ros::Time::now();
    path.header = header;
    header.frame_id = "base_link";
    for(double x = start; x < finish; x += step){
        double y = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.header = header;
        path.poses.push_back(pose);
    }
    return path;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");
    ros::NodeHandle nh;
    ros::Publisher mpcPathPub;
    mpcPathPub = nh.advertise<nav_msgs::Path>("local_path", 1);
    ros::Publisher pathPub;
    pathPub = nh.advertise<nav_msgs::Path>("global_path", 1);
    Eigen::Vector4d coeffs;
    coeffs << 0, 0, 0.1, 0;
    auto globalPath = getPathFromPoly(coeffs);
    mpc::State state{-15, 10, 0, 0, 0, 0};
    mpc::MPC MPC{mpc::getTestTrack(), 30, 0.1};
    std::cout << "x\ty\tpsi\tvel\tcost\n";
    auto refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{100};
    while(ros::ok()){
        auto solution = MPC.solve(state);
        mpc::logSolution(solution, state, "log.txt");
        if (solution.cost == 0) {
            mpc::logSolution(solution, state, "logZeroCost.txt");
        }
        MPC.model(state, solution.u0);
        std::cout << state.x << '\t' << state.y << '\t' << state.psi 
            << '\t' << state.vel << '\t' << solution.cost << std::endl;
        std::cout << "Time: " << solution.computeTime << " [ms]\n";
        mpcPathPub.publish(getPathMsg(solution));
        if (solution.cost < 100 && solution.cost > 0.1){
            state = mpc::State{-15, 20, 0, 0, 0, 0};
        }
        pathPub.publish(globalPath);
        std::this_thread::sleep_until(refresh);
        refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{100};
        std::string s;
        std::cin >> s;
    }
    return 0;
}