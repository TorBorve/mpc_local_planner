#include "mpc_local_planner/RosMpc.h"
#include "mpc_local_planner/utilities.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define AUDIBOT_STEERING_RATIO  17.3

namespace mpc {

    RosMpc::RosMpc() : 
        mpc{getTestTrack(), 30, 0.1}    
    {
        ros::NodeHandle nh;
        inputPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        steeringPub_ = nh.advertise<std_msgs::Float64>("steering_cmd", 1);
        odomSub_ = nh.subscribe("odom", 1, &RosMpc::odomCallback, this);
    }

    void RosMpc::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        static double vel = 0;
        double dt = 0.1;
        State state = toState(*msg);
        state.x += state.vel * cos(state.psi) * dt;
        state.y += state.vel * sin(state.psi) * dt;
        state.psi += msg->twist.twist.angular.z * dt;
        auto result = mpc.solve(state);
        // vel += result.u0.a * dt; 
        vel = 5;
        geometry_msgs::Twist twist;
        twist.linear.x = vel;
        twist.angular.z = rotationSpeed(result.u0.delta, result.mpcHorizon[0].x.vel);
        inputPub_.publish(twist);
        std_msgs::Float64 steeringAngle;
        steeringAngle.data = result.u0.delta * AUDIBOT_STEERING_RATIO;
        steeringPub_.publish(steeringAngle);
        ROS_INFO("refvel: %.2f, carVel: %.2f, steering: %.2f", vel, state.vel, result.u0.delta * 180.0 / M_PI);
    }

    double RosMpc::rotationSpeed(double steeringAngle, double vel) {
        double length = 2.65;
        return tan(steeringAngle) * vel / length;
    }
}