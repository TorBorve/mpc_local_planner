#include "mpc_local_planner/RosMpc.h"
#include "mpc_local_planner/utilities.h"
#include "mpc_local_planner/constants.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define AUDIBOT_STEERING_RATIO  17.3

namespace mpc {

    RosMpc::RosMpc() : 
        mpc{getTestTrack(), MPC_N, MPC_dt}, tfListener_{tfBuffer_}    
    {
        ros::NodeHandle nh;
        inputPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        steeringPub_ = nh.advertise<std_msgs::Float64>("steering_cmd", 1);
        twistSub_ = nh.subscribe("twist", 1, &RosMpc::twistCallback, this);
        actualSteeringSub_ = nh.subscribe("actual_steering_angle", 1, &RosMpc::actualSteeringCallback, this);
    }

    MPCReturn RosMpc::solve() {
        geometry_msgs::TransformStamped tfCar;
        try {
            tfCar = tfBuffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
        } catch (tf2::TransformException& e) {
            ROS_WARN_STREAM("Error thrown: " << e.what());
        }
        State state {
            tfCar.transform.translation.x,
            tfCar.transform.translation.y,
            getYaw(tfCar.transform.rotation),
            current_vel_,
            0,
            0
        };
        Input input{
            0,
            current_steering_angle_
        };
        OptVariables optVars{state, input};
        mpc.model(optVars, input); // get predicted state after calculation is finished

        const auto result = mpc.solve(optVars);
        constexpr double vel = 3;
        geometry_msgs::Twist twist;
        twist.linear.x = vel;
        twist.angular.z = rotationSpeed(result.u0.delta, result.mpcHorizon[0].x.vel);
        inputPub_.publish(twist);
        // std_msgs::Float64 steeringAngle;
        // steeringAngle.data = result.u0.delta * AUDIBOT_STEERING_RATIO;
        // steeringPub_.publish(steeringAngle);
        ROS_INFO("Time: %i [ms]", (int)result.computeTime);
        ROS_INFO("refvel: %.2f, carVel: %.2f, steering: %.2f [deg]", vel, state.vel, result.u0.delta * 180.0 / M_PI);
        return result;
    }

    double RosMpc::rotationSpeed(double steeringAngle, double vel) {
        double length = 2.65;
        return tan(steeringAngle) * vel / length;
    }

    void RosMpc::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_vel_ = length(msg->twist.linear);
    }

    void RosMpc::actualSteeringCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_steering_angle_ = msg->data;
    }
}