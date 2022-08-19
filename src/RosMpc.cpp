#include "mpc_local_planner/RosMpc.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Transform.h>

#include "mpc_local_planner/utilities.h"

namespace mpc {
RosMpc::RosMpc(ros::NodeHandle *nh)
    : controlSys_{util::getParamWarn<double>(*nh, "path_tracking_vel", 2.0), util::getParamWarn<double>(*nh, "parking_vel", 2.0)},
      tfListener_{tfBuffer_},
      nh_{nh} {

    std::string modeStr = util::getParamError<std::string>(*nh_, "mode");
    std::string twistTopic = util::getParamWarn<std::string>(*nh_, "twist_topic", "twist");
    std::string actualSteeringTopic = util::getParamWarn<std::string>(*nh_, "actual_steering_topic", "actual_steering_angle");
    std::string steeringTopic = util::getParamWarn<std::string>(*nh_, "steering_topic", "steering_cmd");
    std::string throttleTopic = util::getParamWarn<std::string>(*nh_, "throttle_topic", "throttle_cmd");
    std::string parkingTopic = util::getParamWarn<std::string>(*nh_, "parking_topic", "goal");
    std::string stopTopic = util::getParamWarn<std::string>(*nh_, "stop_topic", "stop");
    mapFrame_ = util::getParamWarn<std::string>(*nh_, "map_frame", "map");
    carFrame_ = util::getParamWarn<std::string>(*nh_, "car_frame", "base_link");
    steeringRatio_ = util::getParamWarn<double>(*nh_, "steering_ratio", 1.0);

    Mode mode = str2Mode(modeStr);
    if (mode == Mode::Invalid) {
        std::stringstream ss;
        ss << "Invalid mode for mpc: " << modeStr << ". Valid modes are parking, slalom and path_tracking";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error{ss.str()};
    }
    ROS_INFO_STREAM("Initialized with mode: " << toString(mode));
    controlSys_.setMode(mode);
    if (nh_->hasParam("path_topic") && mode == Mode::PathTracking) {
        std::string pathTopic = util::getParamWarn<std::string>(*nh_, "path_topic", "path");
        pathSub_ = nh_->subscribe(pathTopic, 1, &RosMpc::pathCallback, this);
    } else if (!nh_->hasParam("path_topic") && mode == Mode::PathTracking) {
        ROS_WARN("path_topic parameter not specified. Using hardcode internal path.");
    }
    stopPub_ = nh_->advertise<std_msgs::Bool>(stopTopic, 1);
    throttlePub_ = nh_->advertise<std_msgs::Float64>(throttleTopic, 1);
    steeringPub_ = nh_->advertise<std_msgs::Float64>(steeringTopic, 1);
    trackPub_ = nh_->advertise<nav_msgs::Path>("/global_path", 1);
    mpcPathPub_ = nh_->advertise<nav_msgs::Path>("/local_path", 1);
    twistSub_ = nh_->subscribe(twistTopic, 1, &RosMpc::twistCallback, this);
    actualSteeringSub_ = nh_->subscribe(actualSteeringTopic, 1, &RosMpc::actualSteeringCallback, this);
    if (mode == Mode::Parking || mode == Mode::Slalom) {
        poseSub_ = nh_->subscribe(parkingTopic, 1, &RosMpc::poseCallback, this);
    }
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
    std_msgs::Float64 msg;
    msg.data = result.mpcHorizon.at(1).x.throttle;
    throttlePub_.publish(msg);

    std_msgs::Bool stop;
    stop.data = result.stopSignal;
    stopPub_.publish(stop);

    prevThrottle = msg.data;
    msg.data = result.mpcHorizon.at(1).x.delta * steeringRatio_;
    steeringPub_.publish(msg);

    mpcPathPub_.publish(util::getPathMsg(result, mapFrame_, carFrame_));
    trackPub_.publish(util::getPathMsg(controlSys_.getTrack(), mapFrame_, carFrame_));

    // LOG_DEBUG_STREAM(state);
    LOG_DEBUG_STREAM(std::fixed << std::setprecision(2) << result.mpcHorizon.at(1));

    // LOG_DEBUG("Time: %i [ms]", (int)result.computeTime);
    // LOG_DEBUG("carVel: %.2f, steering: %.2f [deg], throttle: %.2f", state.vel,
    // result.mpcHorizon.at(1).x.delta * 180.0 / M_PI, result.mpcHorizon.at(1).x.throttle);
    return result;
}

bool RosMpc::verifyInputs() {
    ros::Duration waitTime{10.0};
    Mode mode = str2Mode(util::getParamError<std::string>(*nh_, "mode"));
    // check if twist publisher is publishing
    std::string twistTopic = util::getParamWarn<std::string>(*nh_, "twist_topic", "twist");
    while (ros::ok() && !ros::topic::waitForMessage<geometry_msgs::TwistStamped>(twistTopic, waitTime)) {
        ROS_WARN("Waiting for twist message. Should be published at the topic: %s", twistTopic.c_str());
    }

    // check if actual steering angle is published
    std::string actualSteeringTopic = util::getParamWarn<std::string>(*nh_, "actual_steering_topic", "actual_steering_topic");
    while (ros::ok() && !ros::topic::waitForMessage<std_msgs::Float64>(actualSteeringTopic, waitTime)) {
        ROS_WARN("Waiting for actual steering angle. Should be published at the topic: %s", actualSteeringTopic.c_str());
    }

    // if path topic parameter is provided. Get the intial path message.
    if (nh_->hasParam("path_topic") && mode == Mode::PathTracking) {
        std::string pathTopic = util::getParamWarn<std::string>(*nh_, "path_topic", "path");
        while (ros::ok()) {
            nav_msgs::Path::ConstPtr firstPath = ros::topic::waitForMessage<nav_msgs::Path>(pathTopic, waitTime);
            if (firstPath != nullptr) {
                controlSys_.setTrack(util::toVector(*firstPath));
                break;
            }
            ROS_WARN("Waiting for path message. Should be published at the topic: %s", pathTopic.c_str());
        }
    }

    if (mode == Mode::Parking) {
        std::string parkingTopic = util::getParamWarn<std::string>(*nh_, "parking_topic", "goal");
        while (ros::ok()) {
            geometry_msgs::PoseStamped::ConstPtr firstPose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(parkingTopic, waitTime);
            if (firstPose != nullptr) {
                controlSys_.setRefPose(firstPose->pose);
                break;
            }
            ROS_WARN("Waiting for pose message for parking spot. Should be published at the topic: %s", parkingTopic.c_str());
        }
    }

    while (ros::ok()) {
        geometry_msgs::TransformStamped tfCar;
        try {
            tfCar = tfBuffer_.lookupTransform(mapFrame_, carFrame_, ros::Time(0), waitTime);
            break;  // break if the previous function did not succed.
        } catch (tf2::TransformException &e) {
            ROS_WARN("Waiting for transfrom from map: \"%s\" to car: \"%s\".\nError message: %s", mapFrame_.c_str(), carFrame_.c_str(),
                     e.what());
        }
    }
    return true;
}

void RosMpc::twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) { currentVel_ = util::length(msg->twist.linear); }

void RosMpc::actualSteeringCallback(const std_msgs::Float64::ConstPtr &msg) { currentSteeringAngle_ = msg->data / steeringRatio_; }

void RosMpc::pathCallback(const nav_msgs::Path::ConstPtr &msg) {
    nav_msgs::Path path = *msg;
    if (msg->header.frame_id != mapFrame_) {
        ROS_WARN_STREAM_THROTTLE(20, "Path has different frame than the MPC use. Therefore a transform will be done.");
        geometry_msgs::TransformStamped tfStampedPathFrame;
        try {
            tfStampedPathFrame = tfBuffer_.lookupTransform(mapFrame_, path.header.frame_id, ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_ERROR_STREAM("Could not get transform from " << mapFrame_ << " to " << path.header.frame_id << ". Error thrown: "
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

void RosMpc::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::Pose pose = msg->pose;

    if (msg->header.frame_id != mapFrame_) {
        tf2::Transform tfGoal;
        tfGoal.setOrigin(tf2::Vector3{pose.position.x, pose.position.y, pose.position.z});
        tfGoal.setRotation(tf2::Quaternion{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});

        geometry_msgs::TransformStamped tfStampedCar;
        try {
            tfStampedCar = tfBuffer_.lookupTransform(mapFrame_, msg->header.frame_id, ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_ERROR_STREAM("Could not get transform from " << mapFrame_ << " to " << msg->header.frame_id
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