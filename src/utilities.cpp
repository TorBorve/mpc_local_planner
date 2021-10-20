#include "mpc_local_planner/utilities.h"

#include <tf2/LinearMath/Quaternion.h>

namespace mpc{
    geometry_msgs::Pose toMsg(const State& state){
        geometry_msgs::Pose pose;
        pose.position.x = state.x;
        pose.position.y = state.y;
        tf2::Quaternion quat;
        quat.setEuler(state.psi, 0, 0);
        pose.orientation.x = quat.getX();
        pose.orientation.y = quat.getY();
        pose.orientation.z = quat.getZ();
        pose.orientation.w = quat.getW();
        return pose;
    }

    nav_msgs::Path getPathMsg(const MPCReturn& solution){
        nav_msgs::Path path;
        std_msgs::Header header;
        header.frame_id = "odom";
        header.stamp = ros::Time::now();
        path.header = header;
        header.frame_id = "base_link";
        path.poses.resize(solution.mpcHorizon.size());
        for(int i = 0; i < path.poses.size(); i++){
            path.poses[i].pose = toMsg(solution.mpcHorizon[i].x);
            path.poses[i].header = header;
        }
        return path;
    }
}