#pragma once
#include "mpc_local_planner/MPC.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>

namespace mpc {

    class RosMpc {
    public:
        RosMpc();

        MPCReturn solve();
    private:

        void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

        void actualSteeringCallback(const std_msgs::Float64::ConstPtr& msg);

        double rotationSpeed(double steeringAngle, double vel);

        MPC mpc;

        ros::Publisher inputPub_;

        ros::Publisher steeringPub_;

        ros::Subscriber twistSub_;

        ros::Subscriber actualSteeringSub_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        double current_vel_ = 0;
        double current_steering_angle_ = 0;
    };
}