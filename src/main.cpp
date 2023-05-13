#include <ros/ros.h>

#include "mpc_local_planner/RosMpc.h"
#include "mpc_local_planner/utilities.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");

    ros::NodeHandle nh{"~"};
    int rate = mpc::util::getParamWarn<int>(nh, "loop_Hz", 30);

    mpc::RosMpc mpc(&nh);
    mpc.verifyInputs();
    ROS_INFO("Topics and transfroms are ok!");
    ros::Duration(2).sleep();  // sleep for two sec

    ros::Rate loopRate(rate);
    while (ros::ok()) {
        ros::spinOnce();
        auto start = std::chrono::high_resolution_clock::now();
        auto ret = mpc.solve();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        LOG_DEBUG("Total time: %ld[ms]", duration.count());
        LOG_DEBUG("Time diff: %f[ms]", duration.count() - ret.computeTime);
        loopRate.sleep();
    }
    return 0;
}