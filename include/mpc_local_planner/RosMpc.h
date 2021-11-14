#pragma once
#include "mpc_local_planner/MPC.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace mpc {

    class RosMpc {
    public:
        RosMpc();
    private:
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

        double rotationSpeed(double steeringAngle, double vel);

        MPC mpc;

        ros::Publisher inputPub_;

        ros::Publisher steeringPub_;

        ros::Subscriber odomSub_;
    };
}