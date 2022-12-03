// #include <ros/ros.h>

#include <rclcpp/rclcpp.hpp>

#include "mpc_local_planner/RosMpc.h"
#include "mpc_local_planner/utilities.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    mpc::util::initLogger();

    auto mpcPtr = std::make_shared<mpc::RosMpc>();
    // LOG_DEBUG_STREAM( )
    mpcPtr->verifyInputs();
    LOG_STREAM("Topics and transfroms are ok!");
    rclcpp::sleep_for(std::chrono::seconds{2});
    rclcpp::Rate rate{std::chrono::milliseconds{33}};
    LOG_STREAM("Starting loop");

    while (rclcpp::ok()) {
        rclcpp::spin_some(mpcPtr);
        auto start = std::chrono::high_resolution_clock::now();
        auto ret = mpcPtr->solve();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        LOG_DEBUG("Total time: %ld[ms]", duration.count());
        LOG_DEBUG("Time diff: %f[ms]", duration.count() - ret.computeTime);
        rate.sleep();
    }

    // ros::init(argc, argv, "MPC");
    // ROS_INFO("MPC node intitialized");

    // ros::NodeHandle nh{"~"};
    // int rate = mpc::util::getParamWarn<int>(nh, "loop_Hz", 30);

    // mpc::RosMpc mpc(&nh);
    // mpc.verifyInputs();
    // ROS_INFO("Topics and transfroms are ok!");
    // ros::Duration(2).sleep();  // sleep for two sec

    // ros::Rate loopRate(rate);
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     auto start = std::chrono::high_resolution_clock::now();
    //     auto ret = mpc.solve();
    //     auto end = std::chrono::high_resolution_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     LOG_DEBUG("Total time: %ld[ms]", duration.count());
    //     LOG_DEBUG("Time diff: %f[ms]", duration.count() - ret.computeTime);
    //     loopRate.sleep();
    // }
    return 0;
}