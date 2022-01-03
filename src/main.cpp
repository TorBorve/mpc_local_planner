#include "mpc_local_planner/RosMpc.h"
#include "mpc_local_planner/constants.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "MPC");
    ROS_INFO("MPC node intitialized");

    ros::NodeHandle nh{"~"};
    int rate = nh.param("loop_Hz", 30);

    mpc::RosMpc mpc(&nh);
    mpc.verifyInputs();
    ROS_INFO("Topics and transfroms are ok!");
    ros::Duration(2).sleep(); // sleep for two sec

    ros::Rate loopRate(rate);
    while(ros::ok()) {
        ros::spinOnce();
        mpc.solve();
        loopRate.sleep();
    }
    return 0;
}