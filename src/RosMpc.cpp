#include "mpc_local_planner/RosMpc.h"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
// #include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Transform.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>

#include <rclcpp/wait_for_message.hpp>

#include "mpc_local_planner/utilities.h"

namespace mpc {
RosMpc::RosMpc()
    : Node{"mpc_local_planner",
           rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)},
      controlSys_{this->get_parameter("path_tracking_vel").get_parameter_value().get<double>(),
                  this->get_parameter("parking_vel").get_parameter_value().get<double>()} {
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    std::string modeStr = util::getParamError<std::string>(*this, "mode");
    std::string twistTopic = util::getParamWarn<std::string>(*this, "twist_topic", "twist");
    std::string actualSteeringTopic = util::getParamWarn<std::string>(*this, "actual_steering_topic", "actual_steering_angle");
    std::string steeringTopic = util::getParamWarn<std::string>(*this, "steering_topic", "steering_cmd");
    std::string throttleTopic = util::getParamWarn<std::string>(*this, "throttle_topic", "throttle_cmd");
    std::string parkingTopic = util::getParamWarn<std::string>(*this, "parking_topic", "goal");
    std::string stopTopic = util::getParamWarn<std::string>(*this, "stop_topic", "stop");
    mapFrame_ = util::getParamWarn<std::string>(*this, "map_frame", "map");
    carFrame_ = util::getParamWarn<std::string>(*this, "car_frame", "base_link");
    steeringRatio_ = util::getParamWarn<double>(*this, "steering_ratio", 1.0);

    Mode mode = str2Mode(modeStr);
    if (mode == Mode::Invalid) {
        std::stringstream ss;
        ss << "Invalid mode for mpc: " << modeStr << ". Valid modes are parking, slalom and path_tracking";
        RCLCPP_ERROR_STREAM(this->get_logger(), ss.str());
        throw std::runtime_error{ss.str()};
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized with mode: " << toString(mode));
    controlSys_.setMode(mode);

    using std::placeholders::_1;
    rclcpp::Parameter temp;
    if (this->get_parameter_or("path_topic", temp, rclcpp::Parameter{"path_topic", "INVALID"}) && mode == Mode::PathTracking) {
        std::string pathTopic = util::getParamWarn<std::string>(*this, "path_topic", "path");
        pathSub_ = this->create_subscription<nav_msgs::msg::Path>(pathTopic, 1, std::bind(&RosMpc::pathCallback, this, _1));
    } else if (!this->get_parameter_or("path_topic", temp, rclcpp::Parameter{"path_topic", "INVALID"}) && mode == Mode::PathTracking) {
        RCLCPP_WARN(this->get_logger(), "path_topic parameter not specified. Using hardcode internal path.");
    }

    stopPub_ = this->create_publisher<std_msgs::msg::Bool>(stopTopic, 1);
    throttlePub_ = this->create_publisher<example_interfaces::msg::Float64>(throttleTopic, 1);
    steeringPub_ = this->create_publisher<example_interfaces::msg::Float64>(steeringTopic, 1);
    trackPub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
    mpcPathPub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 1);
    twistSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(twistTopic, 1, std::bind(&RosMpc::twistCallback, this, _1));
    actualSteeringSub_ = this->create_subscription<example_interfaces::msg::Float64>(actualSteeringTopic, 1, std::bind(&RosMpc::actualSteeringCallback, this, _1));
    if (mode == Mode::Parking || mode == Mode::Slalom) {
        poseSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(parkingTopic, 1, std::bind(&RosMpc::poseCallback, this, _1));
    }
}

MPCReturn RosMpc::solve() {
    static double prevThrottle = 0;

    geometry_msgs::msg::TransformStamped tfCar;
    try {
        // get position of vehicle
        tfCar = tfBuffer_->lookupTransform(mapFrame_, carFrame_, tf2::TimePointZero);
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform from " << mapFrame_ << " to " << carFrame_ << ". Error thrown: " << e.what());
        return MPCReturn{};
    }
    State state{tfCar.transform.translation.x,
                tfCar.transform.translation.y,
                util::getYaw(tfCar.transform.rotation),
                currentVel_,
                currentSteeringAngle_,
                prevThrottle};

    // solve mpc
    const auto result = controlSys_.solve(state, util::getPitch(tfCar.transform.rotation));

    if (result.mpcHorizon.size() < 1) {
        return result;
    }

    // publish inputs
    example_interfaces::msg::Float64 msg;
    msg.data = result.mpcHorizon.at(1).x.throttle;
    throttlePub_->publish(msg);

    std_msgs::msg::Bool stop;
    stop.data = result.stopSignal;
    stopPub_->publish(stop);

    prevThrottle = msg.data;
    msg.data = result.mpcHorizon.at(1).x.delta * steeringRatio_;
    steeringPub_->publish(msg);

    mpcPathPub_->publish(util::getPathMsg(result, mapFrame_, carFrame_, *this));
    trackPub_->publish(util::getPathMsg(controlSys_.getTrack(), mapFrame_, carFrame_, *this));
    return result;
}

bool RosMpc::verifyInputs() {
    std::chrono::seconds waitTime{10};

    Mode mode = str2Mode(util::getParamError<std::string>(*this, "mode"));
    // check if twist publisher is publishing
    std::string twistTopic = util::getParamWarn<std::string>(*this, "twist_topic", "twist");
    auto waitNode = std::make_shared<rclcpp::Node>("mpc_wait_for_message");

    auto twistMsg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    while (!rclcpp::wait_for_message(*twistMsg, waitNode, twistTopic, waitTime)){
        RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for twist message. Should be published at the topic: " << twistTopic);
    }
    twistCallback(twistMsg);

    // check if actual steering angle is published
    std::string actualSteeringTopic = util::getParamWarn<std::string>(*this, "actual_steering_topic", "actual_steering_topic");

    auto steeringMsg = std::make_shared<example_interfaces::msg::Float64>();
    while (!rclcpp::wait_for_message(*steeringMsg, waitNode, actualSteeringTopic, waitTime)){
        RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for actual steering angle. Should be published at the topic: " << actualSteeringTopic);
    }
    actualSteeringCallback(steeringMsg);

    // if path topic parameter is provided. Get the intial path message.
    rclcpp::Parameter temp;
    if (this->get_parameter_or("path_topic", temp, rclcpp::Parameter{"path_topic", "INVALID"}) && mode == Mode::PathTracking) {
        std::string pathTopic = util::getParamWarn<std::string>(*this, "path_topic", "path");
        while (rclcpp::ok()) {
            nav_msgs::msg::Path firstPath;
            if (rclcpp::wait_for_message(firstPath, waitNode, pathTopic, waitTime)){
                controlSys_.setTrack(util::toVector(firstPath));
                break;
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for path message. Should be published at the topic: " << pathTopic);
        }
    }

    if (mode == Mode::Parking) {
        std::string parkingTopic = util::getParamWarn<std::string>(*this, "parking_topic", "goal");
        while (rclcpp::ok()) {
            geometry_msgs::msg::PoseStamped firstPose;
            if (rclcpp::wait_for_message(firstPose, waitNode, parkingTopic, waitTime)){
                controlSys_.setRefPose(firstPose.pose);
                break;
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for pose message for parking spot. Should be published at the topic: " << parkingTopic);
        }
    }

    while (rclcpp::ok()) {
        geometry_msgs::msg::TransformStamped tfCar;
        try {
            // get position of vehicle
            tfCar = tfBuffer_->lookupTransform(mapFrame_, carFrame_, tf2::TimePointZero, waitTime);
            break;
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for transform from " << mapFrame_ << " to " << carFrame_ << ". Error thrown: " << e.what());
        }
    }
    return true;
}

void RosMpc::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { currentVel_ = util::length(msg->twist.linear); }

void RosMpc::actualSteeringCallback(const example_interfaces::msg::Float64::SharedPtr msg) { currentSteeringAngle_ = msg->data / steeringRatio_; }

void RosMpc::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    nav_msgs::msg::Path path = *msg;
    if (msg->header.frame_id != mapFrame_) {
        geometry_msgs::msg::TransformStamped tfStampedPathFrame;
        try {
            tfStampedPathFrame = tfBuffer_->lookupTransform(mapFrame_, path.header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform from " << mapFrame_ << " to " << path.header.frame_id << ". Error thrown: "
                                                             << e.what() << "\nNeed transform for calculating pose of path used in MPC.");
            return;
        }

        tf2::Transform tfPath;
        auto &trans = tfStampedPathFrame.transform.translation;
        tfPath.setOrigin(tf2::Vector3{trans.x, trans.y, trans.z});
        auto &q = tfStampedPathFrame.transform.rotation;
        tfPath.setRotation(tf2::Quaternion{q.x, q.y, q.z, q.w});

        for (auto &stampedPose : path.poses) {
            tf2::Transform tfOriginal;
            auto &pose = stampedPose.pose;
            tfOriginal.setOrigin(tf2::Vector3{pose.position.x, pose.position.y, pose.position.z});
            tfOriginal.setRotation(tf2::Quaternion{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});

            tf2::Transform tfNew = tfPath * tfOriginal;
            auto rot = tfNew.getRotation();
            pose.orientation.x = rot.getX();
            pose.orientation.y = rot.getY();
            pose.orientation.z = rot.getZ();
            pose.orientation.w = rot.getW();

            auto org = tfNew.getOrigin();
            pose.position.x = org.getX();
            pose.position.y = org.getY();
            pose.position.z = org.getZ();
        }
    }
    controlSys_.setTrack(util::toVector(path));
}

void RosMpc::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    geometry_msgs::msg::Pose pose = msg->pose;

    if (msg->header.frame_id != mapFrame_) {
        tf2::Transform tfGoal;
        tfGoal.setOrigin(tf2::Vector3{pose.position.x, pose.position.y, pose.position.z});
        tfGoal.setRotation(tf2::Quaternion{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});

        geometry_msgs::msg::TransformStamped tfStampedCar;
        try {
            tfStampedCar = tfBuffer_->lookupTransform(mapFrame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform from " << mapFrame_ << " to " << msg->header.frame_id
                                                             << ". Error thrown: " << e.what()
                                                             << "\nNeed transform for calculating positon of new parking pose for car");
            return;
        }
        tf2::Transform tfCar;
        auto &trans = tfStampedCar.transform.translation;
        tfCar.setOrigin(tf2::Vector3{trans.x, trans.y, trans.z});
        auto &q = tfStampedCar.transform.rotation;
        tfCar.setRotation(tf2::Quaternion{q.x, q.y, q.z, q.w});

        tfCar *= tfGoal;

        auto rot = tfCar.getRotation();
        pose.orientation.x = rot.getX();
        pose.orientation.y = rot.getY();
        pose.orientation.z = rot.getZ();
        pose.orientation.w = rot.getW();

        auto org = tfCar.getOrigin();
        pose.position.x = org.getX();
        pose.position.y = org.getY();
        pose.position.z = org.getZ();
    }
    controlSys_.setRefPose(pose);
}
}  // namespace mpc