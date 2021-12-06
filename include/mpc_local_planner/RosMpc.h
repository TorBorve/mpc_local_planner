#pragma once
#include "mpc_local_planner/MPC.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>

namespace mpc {

    /// @brief class for mpc with ROS interface
    class RosMpc {
    public:
        /// @brief default constructor for RosMpc class
        RosMpc();
        
        /// @brief solve function for mpc. Calls internal mpc class with latest values for state.
        MPCReturn solve();

        /// @brief Check if all the topics and frames needed for the mpc exist and are active.
        /// @return True when all topics and frames are active.
        bool verifyInputs() const;
    private:

        /// @brief callback function for subscriber to twist topic. 
        /// @param[in] msg the motion of the car (twist message)
        void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

        /// @brief callback function for subscriber to steering angle.
        /// @param[in] msg the current steering angle of the var.
        void actualSteeringCallback(const std_msgs::Float64::ConstPtr& msg);

        /// @brief uses bicycle model to calculate the resulting angular velocity given the speed and steering angle
        /// @param[in] steeringAngle the wanted steering angle of the car.
        /// @param[in] vel the current speed of the car.
        /// @return resulting angular velocity [rad / s]
        double rotationSpeed(double steeringAngle, double vel);

        /// @brief mpc class that solves the problem given our state and desired trajectory.
        MPC mpc;

        /// @brief publisher for the inputs. Given as e twist message. linear speed and angular speed.
        ros::Publisher inputPub_;

        /// @brief publicher for the steering angle. 
        ros::Publisher steeringPub_;

        /// @brief subscriber to the twist message send by the car. 
        ros::Subscriber twistSub_;

        /// @brief subscriber to the steering angle of the car.
        ros::Subscriber actualSteeringSub_;

        /// @brief buffer for tf. Used to get the position of the car.
        tf2_ros::Buffer tfBuffer_;

        /// @brief tf listener
        tf2_ros::TransformListener tfListener_;

        /// @brief the latest velocity recived from car.
        double current_vel_ = 0;

        /// @brief the latest steering angle recived from car
        double current_steering_angle_ = 0;

        /// @brief topic name for where the twist message is published
        std::string twistTopic_ = "twist";

        /// @brief topic name for where the actual steering angle is published
        std::string actualSteeringTopic_ = "actual_steering_angle";

        /// @brief name of "origin" frame. Typically odom or map
        std::string mapFrame_ = "odom";

        /// @brief name of frame where the car is located
        std::string carFrame_ = "base_footprint";
    };
}