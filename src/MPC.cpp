#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/bounds.h"
#include "mpc_local_planner/utilities.h"
#include "mpc_local_planner/AcadosSolver.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

#include <fstream>

namespace mpc
{
    MPC::MPC(const std::vector<Point> &track, size_t N, double dt, Bound steeringAngle, double maxSteeringRotationSpeed, 
            double wheelbase) : track_{track}, N_{N}, dt_{dt}, steeringAngle_{steeringAngle}, 
                                maxSteeringRotationSpeed_{maxSteeringRotationSpeed}, wheelbase_{wheelbase}
    {
        ros::NodeHandle nh;
        trackPub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
        mpcPathPub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
        polynomPub_ = nh.advertise<nav_msgs::Path>("interpolated_path", 1);
    }

    MPCReturn MPC::solve(const OptVariables &optVars)
    {
        const State &state = optVars.x;

        pubTf(state);

        double rotation;
        Eigen::Vector4d coeffs;
        calcCoeffs(state, rotation, coeffs);

        State transformedState{0, 0, rotation, state.vel, 0, 0};
        calcState(transformedState, coeffs);
        OptVariables transformedOptVar{transformedState, optVars.u};

        auto result = solve(transformedOptVar, coeffs);

        const double rotAngle = state.psi - rotation;
        const double sinRot = sin(rotAngle);
        const double cosRot = cos(rotAngle);
        auto &x = result.mpcHorizon;
        for (unsigned int i = 0; i < x.size(); i++)
        {
            // rotate back
            double dx = x[i].x.x;
            double dy = x[i].x.y;
            x[i].x.x = dx * cosRot - dy * sinRot;
            x[i].x.y = dx * sinRot + dy * cosRot;

            // // shift coordinates
            x[i].x.x += state.x;
            x[i].x.y += state.y;
        }

        auto polyPath = getPathMsg(coeffs);
        auto &points = polyPath.poses;
        for (unsigned int i = 0; i < points.size(); i++)
        {
            double dx = points[i].pose.position.x;
            double dy = points[i].pose.position.y;
            points[i].pose.position.x = dx * cosRot - dy * sinRot;
            points[i].pose.position.y = dx * sinRot + dy * cosRot;

            points[i].pose.position.x += state.x;
            points[i].pose.position.y += state.y;
        }
        polynomPub_.publish(polyPath);
        trackPub_.publish(getPathMsg(track_));
        mpcPathPub_.publish(getPathMsg(result));

        return result;
    }

    MPCReturn MPC::solve(const OptVariables &optVars, const Eigen::Vector4d &coeffs)
    {
        static AcadosSolver solver{optVars};
        return solver.solve(optVars, coeffs);
    }

    void MPC::calcCoeffs(const State &state, double &rotation, Eigen::Vector4d &coeffs) const
    {
        size_t start, end;
        getTrackSection(start, end, state);

        double minCost = 1e19;
        for (double rot = -M_PI_2; rot < 0; rot += M_PI_2 / 3)
        {
            double curCost =  1e19;
            Eigen::Vector4d curCoeffs = interpolate(state, rot, start, end, curCost);
            if (curCost < minCost)
            {
                minCost = curCost;
                coeffs = curCoeffs;
                rotation = rot;
            }
        }
        return;
    }

    void MPC::calcState(State &state, const Eigen::Vector4d &coeffs) const
    {
        state.cte = state.y - polyEval(state.x, coeffs);
        double dy = coeffs[1] + 2 * state.x * coeffs[2] + 3 * coeffs[3] * state.x * state.x;
        state.epsi = state.psi - atan2(dy, 1);
        return;
    }

    Eigen::Vector4d MPC::interpolate(const State &state, double rotation, size_t start, size_t end, double &cost) const
    {
        Eigen::VectorXd xVals(end - start);
        Eigen::VectorXd yVals(end - start);
        double angle = rotation - state.psi;

        for (unsigned int i = start; i < end; i++)
        {
            // shift points so that the state is in the origo
            double dx = track_[i].x - state.x;
            double dy = track_[i].y - state.y;

            // rotate points so that state.psi = 0 in the new refrence frame
            xVals[i - start] = dx * cos(angle) - dy * sin(angle);
            yVals[i - start] = dx * sin(angle) + dy * cos(angle);
        }

        auto coeffs = polyfit(xVals, yVals, 3);
        assert(coeffs.size() == 4);

        cost = 0;
        for (unsigned int i = 0; i < yVals.size(); i++)
        {
            cost += distSqrd(yVals[i] - polyEval(xVals[i], coeffs), 0);
        }
        return coeffs;
    }

    void MPC::model(OptVariables &optVars, const Input &u)
    {
        model(optVars, u, this->dt_);
    }

    void MPC::model(OptVariables &optVars, const Input &u, double dt)
    {
        ROS_WARN("model is out dated and needs to be updated...");
        const double maxInc = maxSteeringRotationSpeed_ * dt_;
        State &state = optVars.x;
        double delta = u.delta;
        if (delta < optVars.u.delta - maxInc)
        {
            delta = optVars.u.delta - maxInc;
            ROS_WARN("Unable to turn wheels fast enough");
        }
        else if (delta > optVars.u.delta + maxInc)
        {
            delta = optVars.u.delta + maxInc;
            ROS_WARN("Unable to turn wheels fast enough");
        }
        optVars.u.delta = delta;
        state.x += state.vel * cos(state.psi) * dt;
        state.y += state.vel * sin(state.psi) * dt;
        state.psi += state.vel * tan(delta) / wheelbase_ * dt;
        state.vel += u.throttle * dt;
    }

    void MPC::getTrackSection(size_t &start, size_t &end, const State &state) const
    {
        double maxLen = 15;
        double minDistSqrd = distSqrd(state.x - track_[0].x, state.y - track_[0].y);
        size_t minIndex = 0;
        for (unsigned int i = 1; i < track_.size(); i++)
        {
            double curDistSqrd = distSqrd(state.x - track_[i].x, state.y - track_[i].y);
            if (curDistSqrd < minDistSqrd)
            {
                minDistSqrd = curDistSqrd;
                minIndex = i;
            }
        }

        double len = 0;
        size_t frontIndex = minIndex;
        while (len < maxLen && frontIndex < track_.size() - 1)
        {
            frontIndex++;
            len += sqrt(distSqrd(track_[frontIndex].x - track_[frontIndex - 1].x, track_[frontIndex].y - track_[frontIndex - 1].y));
        }
        start = minIndex;
        end = frontIndex;
        if (end - start < 4)
        {
            end = start + 4;
        }
        if (end >= track_.size())
        {
            start = 0;
            end = start + 4;
        }
        assert(end < track_.size());
    }

    void MPC::pubTf(const State &state) const
    {
        static tf2_ros::TransformBroadcaster br;

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "mpc_base_link";
        transformStamped.transform.translation.x = state.x;
        transformStamped.transform.translation.y = state.y;
        transformStamped.transform.translation.z = 0.5;
        tf2::Quaternion q;
        q.setRPY(0, 0, state.psi);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
    }
}