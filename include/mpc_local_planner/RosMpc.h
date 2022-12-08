#ifndef MPC_ROS_MPC_H_
#define MPC_ROS_MPC_H_

#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// #include "std_msgs/msg/float64.hpp"

#include "mpc_local_planner/ControlSys.h"

namespace mpc {

/// @brief class for mpc with ROS interface
class RosMpc : public rclcpp::Node {
   public:
    /// @brief default constructor for RosMpc class
    /// @param[in] nh pointer to NodeHandle with access to private variables .
    RosMpc();

    /// @brief solve function for mpc. Calls internal mpc class with latest values for state.
    MPCReturn solve();

    /// @brief Check if all the topics and frames needed for the mpc exist and are active.
    /// @return True when all topics and frames are active.
    bool verifyInputs();

   private:
    /// @brief callback function for subscriber to twist topic.
    /// @param[in] msg the motion of the car (twist message)
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    /// @brief callback function for subscriber to steering angle.
    /// @param[in] msg the current steering angle of the var.
    void actualSteeringCallback(const example_interfaces::msg::Float64::SharedPtr msg);

    /// @brief callback function for subscriber to path topic.
    /// @param[in] msg the new path message. The path we want to follow.
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    /// @brief callback function for subscriber to pose topic
    /// @param[in] msg the new reference pose the car should move to.
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /// @brief mpc class that solves the problem given our state and desired trajectory.
    ControlSys controlSys_;

    /// @brief publisher for the steering angle.
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr steeringPub_;

    /// @brief publisher for throttle value
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr throttlePub_;

    /// @brief publisher for the desired trajectory
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trackPub_;

    /// @brief publisher for the mpc trajectory solution
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpcPathPub_;

    /// @brief pulisher for the stop signal.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stopPub_;

    /// @brief subscriber to the twist message send by the car.
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twistSub_;

    /// @brief subscriber to the steering angle of the car.
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr actualSteeringSub_;

    /// @brief subscriber to path topic. The path we want to follow.
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub_;

    /// @brief subscriber to the pose topic. The pose we want the car to move to.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;

    /// @brief buffer for tf. Used to get the position of the car.
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

    /// @brief tf listener
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    /// @brief the latest velocity recived from car.
    double currentVel_ = 0;

    /// @brief the latest steering angle recived from car
    double currentSteeringAngle_ = 0;

    /// @brief name of "origin" frame. Typically odom or map
    std::string mapFrame_;

    /// @brief name of frame where the car is located
    std::string carFrame_;

    /// @brief Ratio between angle on the wheels and the steering wheel. steeringRatio = "steering
    /// wheel angle" / "wheel angle"
    double steeringRatio_;
};
}  // namespace mpc

#endif