#include "mpc_local_planner/RosMpc.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");

    mpc::RosMpc mpc;
    ros::Rate loopRate(20);
    ros::Duration(2).sleep(); // sleep for two sec
    while(ros::ok()) {
        ros::spinOnce();
        mpc.solve();
        loopRate.sleep();
    }
    return 0;
}