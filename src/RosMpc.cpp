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
        twistSub_ = nh.subscribe(twistTopic_, 1, &RosMpc::twistCallback, this);
        actualSteeringSub_ = nh.subscribe(actualSteeringTopic_, 1, &RosMpc::actualSteeringCallback, this);
    }

    MPCReturn RosMpc::solve() {
        geometry_msgs::TransformStamped tfCar;
        try {
            tfCar = tfBuffer_.lookupTransform(mapFrame_, carFrame_, ros::Time(0));
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
        // mpc.model(optVars, input); // get predicted state after calculation is finished

        const auto result = mpc.solve(optVars);
        constexpr double vel = 8;
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

    bool RosMpc::verifyInputs() const {
        double waitTime = 10.0;

        // check if twist publisher is publishing
        while (ros::ok() && !ros::topic::waitForMessage<geometry_msgs::TwistStamped>(twistTopic_, ros::Duration{waitTime})) {
            ROS_WARN("Waiting for twist message. Should be published at the topic: %s", twistTopic_.c_str());
        }

        // cheeck if actual steering angle is published
        while (ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(actualSteeringTopic_, ros::Duration{waitTime})) {
            ROS_WARN("Waiting for actual steering angle. Should be publishd at the topic: %s", actualSteeringTopic_.c_str());
        }

        while (ros::ok()) {
            bool ok = true; // if the transform was recived
            geometry_msgs::TransformStamped tfCar;
            try {
                tfCar = tfBuffer_.lookupTransform(mapFrame_, carFrame_, ros::Time(0), ros::Duration{waitTime});
            } catch (tf2::TransformException& e) {
                ROS_WARN("Waiting for transfrom from map to car.\n\t Error message: %s", e.what());
                ok = false;
            }
            if (ok) {
                break;
            }
        }
        return true;
    }

    double RosMpc::rotationSpeed(double steeringAngle, double vel) {
        return tan(steeringAngle) * vel / MPC_WHEELBASE;
    }

    void RosMpc::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_vel_ = length(msg->twist.linear);
    }

    void RosMpc::actualSteeringCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_steering_angle_ = msg->data;
    }
}