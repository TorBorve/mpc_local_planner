#include "mpc_local_planner/RosMpc.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");

    mpc::RosMpc mpc;
    ros::Rate loopRate(10);
    while(ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}