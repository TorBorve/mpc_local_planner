#ifndef MPC_ROS_MPC_H_
#define MPC_ROS_MPC_H_

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>

#include "mpc_local_planner/ControlSys.h"

namespace mpc {

/// @brief class for mpc with ROS interface
class RosMpc {
   public:
    /// @brief default constructor for RosMpc class
    /// @param[in] nh pointer to NodeHandle with access to private variables .
    RosMpc(ros::NodeHandle *nh);

    /// @brief solve function for mpc. Calls internal mpc class with latest values for state.
    MPCReturn solve();

    /// @brief Check if all the topics and frames needed for the mpc exist and are active.
    /// @return True when all topics and frames are active.
    bool verifyInputs();

   private:
    /// @brief callback function for subscriber to twist topic.
    /// @param[in] msg the motion of the car (twist message)
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    /// @brief callback function for subscriber to steering angle.
    /// @param[in] msg the current steering angle of the var.
    void actualSteeringCallback(const std_msgs::Float64::ConstPtr &msg);

    /// @brief callback function for subscriber to path topic.
    /// @param[in] msg the new path message. The path we want to follow.
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /// @brief checks if the desired parameters is given to the MPC class.
    /// @param[in] nh pointer to Nodehandle with access to the parameters
    /// @return true if all parameters was defined. False otherwise.
    /// @throw runtime exception if one or more critical parmeters is not defined.
    bool verifyParamsForMPC(ros::NodeHandle *nh) const;

    /// @brief mpc class that solves the problem given our state and desired trajectory.
    ControlSys controlSys_;

    /// @brief publisher for the steering angle.
    ros::Publisher steeringPub_;

    /// @brief publisher for throttle value
    ros::Publisher throttlePub_;

    /// @brief publisher for the desired trajectory
    ros::Publisher trackPub_;

    /// @brief publisher for the mpc trajectory solution
    ros::Publisher mpcPathPub_;

    ros::Publisher stopPub_;

    /// @brief subscriber to the twist message send by the car.
    ros::Subscriber twistSub_;

    /// @brief subscriber to the steering angle of the car.
    ros::Subscriber actualSteeringSub_;

    /// @brief subscriber to path topic. The path we want to follow.
    ros::Subscriber pathSub_;

    ros::Subscriber poseSub_;

    /// @brief buffer for tf. Used to get the position of the car.
    tf2_ros::Buffer tfBuffer_;

    /// @brief tf listener
    tf2_ros::TransformListener tfListener_;

    /// @brief pointer to nodehandle with access to private parameters.
    ros::NodeHandle *nh_;

    /// @brief the latest velocity recived from car.
    double currentVel_ = 0;

    /// @brief the latest steering angle recived from car
    double currentSteeringAngle_ = 0;

    /// @brief name of "origin" frame. Typically odom or map
    std::string mapFrame_;

    /// @brief name of frame where the car is located
    std::string carFrame_;

    /// @brief frequency that the mpc is run at.
    double loopHz_;

    /// @brief time between steps in the mpc calculations.
    double mpcDt_;

    /// @brief Ratio between angle on the wheels and the steering wheel. steeringRatio = "steering wheel angle" / "wheel angle"
    double steeringRatio_;

    /// @brief the mode of the control system. This can be parking or path tracking.
    ControlSys::Mode mode_;
};
}  // namespace mpc

#endif