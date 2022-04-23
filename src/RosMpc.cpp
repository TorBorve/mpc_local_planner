#include "mpc_local_planner/RosMpc.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "mpc_local_planner/utilities.h"

namespace mpc {

RosMpc::RosMpc(ros::NodeHandle *nh) : mpc{getTestTrack()}, tfListener_{tfBuffer_}, nh_{nh} {
    if (!verifyParamsForMPC(nh)) {
        ROS_WARN("One or more parameters for the mpc is not specified. Default values is therefore used.");
    }

    std::string twistTopic = nh_->param<std::string>("twist_topic", "twist");
    std::string actualSteeringTopic = nh_->param<std::string>("actual_steering_topic", "actual_steering_angle");
    std::string commandTopic = nh_->param<std::string>("command_topic", "car_cmd");
    std::string steeringTopic = nh_->param<std::string>("steering_topic", "steering_cmd");
    std::string throttleTopic = nh_->param<std::string>("throttle_topic", "throttle_cmd");
    mapFrame_ = nh->param<std::string>("map_frame", "map");
    carFrame_ = nh->param<std::string>("car_frame", "base_link");
    loopHz_ = nh->param<double>("loop_Hz", 30);
    mpcDt_ = nh->param<double>("mpc_dt", 0.2);
    steeringRatio_ = nh->param<double>("steering_ratio", 1.0);

    if (nh_->hasParam("path_topic")) {
        std::string pathTopic = nh_->param<std::string>("path_topic", "/path");
        pathSub_ = nh->subscribe(pathTopic, 1, &RosMpc::pathCallback, this);
    } else {
        ROS_WARN("path_topic parameter not specified. Using hardcode internal path.");
        trackPub_ = nh->advertise<nav_msgs::Path>("/global_path", 1);
    }
    throttlePub_ = nh->advertise<std_msgs::Float64>(throttleTopic, 1);
    steeringPub_ = nh->advertise<std_msgs::Float64>(steeringTopic, 1);
    mpcPathPub_ = nh->advertise<nav_msgs::Path>("/local_path", 1);
    twistSub_ = nh->subscribe(twistTopic, 1, &RosMpc::twistCallback, this);
    actualSteeringSub_ = nh->subscribe(actualSteeringTopic, 1, &RosMpc::actualSteeringCallback, this);
}

MPCReturn RosMpc::solve() {
    static double prevThrottle = 0;

    geometry_msgs::TransformStamped tfCar;
    try {
        // get position of vehicle
        tfCar = tfBuffer_.lookupTransform(mapFrame_, carFrame_, ros::Time(0));
    } catch (tf2::TransformException &e) {
        ROS_ERROR_STREAM("Could not get transform from " << mapFrame_ << " to " << carFrame_ << ". Error thrown: " << e.what());
        return MPCReturn{};
    }
    State state{
        tfCar.transform.translation.x,
        tfCar.transform.translation.y,
        getYaw(tfCar.transform.rotation),
        currentVel_,
        currentSteeringAngle_,
        prevThrottle};

    // solve mpc
    const auto result = mpc.solve(state, getPitch(tfCar.transform.rotation));

    // publish inputs
    std_msgs::Float64 msg;
    msg.data = result.mpcHorizon.at(1).x.throttle;
    throttlePub_.publish(msg);
    prevThrottle = msg.data;
    msg.data = result.mpcHorizon.at(1).x.delta * steeringRatio_;
    steeringPub_.publish(msg);

    mpcPathPub_.publish(getPathMsg(result, mapFrame_, carFrame_));
    if (!nh_->hasParam("path_topic")) {
        trackPub_.publish(getPathMsg(mpc.getTrack(), mapFrame_, carFrame_));
    }

    LOG_DEBUG("Time: %i [ms]", (int)result.computeTime);
    LOG_DEBUG("carVel: %.2f, steering: %.2f [deg], throttle: %.2f", state.vel, result.mpcHorizon.at(1).x.delta * 180.0 / M_PI, result.mpcHorizon.at(1).x.throttle);
    return result;
}

bool RosMpc::verifyInputs() {
    ros::Duration waitTime{10.0};
    ros::Duration{0.1}.sleep();
    // check if twist publisher is publishing
    std::string twistTopic = nh_->param<std::string>("twist_topic", "/twist");
    while (ros::ok() && !ros::topic::waitForMessage<geometry_msgs::TwistStamped>(twistTopic, waitTime)) {
        ROS_WARN("Waiting for twist message. Should be published at the topic: %s", twistTopic.c_str());
    }

    // check if actual steering angle is published
    std::string actualSteeringTopic = nh_->param<std::string>("actual_steering_topic", "/actual_steering_angle");
    while (ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(actualSteeringTopic, waitTime)) {
        ROS_WARN("Waiting for actual steering angle. Should be published at the topic: %s", actualSteeringTopic.c_str());
    }

    // if path topic parameter is provided. Get the intial path message.
    if (nh_->hasParam("path_topic")) {
        std::string pathTopic = nh_->param<std::string>("path_topic", "/path");
        while (ros::ok()) {
            nav_msgs::Path::ConstPtr firstPath = ros::topic::waitForMessage<nav_msgs::Path>(pathTopic, waitTime);
            if (firstPath != nullptr) {
                mpc.setTrack(toVector(*firstPath));
                break;
            }
            ROS_WARN("Waiting for path message. Should be published at the topic: %s", pathTopic.c_str());
        }
    }

    while (ros::ok()) {
        geometry_msgs::TransformStamped tfCar;
        try {
            tfCar = tfBuffer_.lookupTransform(mapFrame_, carFrame_, ros::Time(0), waitTime);
            break;  // break if the previous function did not succed.
        } catch (tf2::TransformException &e) {
            ROS_WARN("Waiting for transfrom from map: \"%s\" to car: \"%s\".\nError message: %s", mapFrame_.c_str(), carFrame_.c_str(), e.what());
        }
    }
    return true;
}

void RosMpc::twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    currentVel_ = length(msg->twist.linear);
}

void RosMpc::actualSteeringCallback(const std_msgs::Float64::ConstPtr &msg) {
    currentSteeringAngle_ = msg->data / steeringRatio_;
}

void RosMpc::pathCallback(const nav_msgs::Path::ConstPtr &msg) {
    mpc.setTrack(toVector(*msg));
}

bool RosMpc::verifyParamsForMPC(ros::NodeHandle *nh) const {
    bool ok = true;
    ok &= hasParamError(nh, "mpc_N");
    ok &= hasParamError(nh, "mpc_dt");
    ok &= hasParamWarn(nh, "max_steering_angle");
    ok &= hasParamWarn(nh, "max_steering_rotation_speed");
    ok &= hasParamWarn(nh, "wheelbase");
    ok &= hasParamWarn(nh, "twist_topic");
    ok &= hasParamWarn(nh, "actual_steering_topic");
    ok &= hasParamWarn(nh, "map_frame");
    ok &= hasParamWarn(nh, "car_frame");
    ok &= hasParamWarn(nh, "loop_Hz");
    return ok;
}
}  // namespace mpc