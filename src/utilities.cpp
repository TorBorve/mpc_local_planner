#include "mpc_local_planner/utilities.h"

#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <iomanip>

namespace mpc{
    geometry_msgs::Pose toMsg(const State& state) {
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

    nav_msgs::Path getPathMsg(const MPCReturn& solution) {
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

    CppAD::vector<double> toCppAD(const std::vector<double>& vec){
        CppAD::vector<double> cppADVec(vec.size());
        for (int i = 0; i < vec.size(); i++) {
            cppADVec[i] = vec[i];
        }
        return cppADVec;
    }

    void logSolution(const MPCReturn& solution, const State& curState, const std::string& filename) {
        std::ofstream outFile(filename);
        if (!outFile) {
            std::cout << "could not write to file log.txt\n";
            return;
        }
        outFile << "Log for mpc solution\n";
        outFile << "Cost: " << solution.cost << std::endl;
        outFile << "Current State:\n";
        outFile << "x\ty\tpsi\tvel\n";
        outFile << curState.x << '\t' << curState.y << '\t' << curState.psi << '\t' << curState.vel << '\n';
        outFile << "State and input from horizon:\n";
        outFile << "x\ty\tpsi\tvel\tcte\tepsi\ta\tdelta\n";
        outFile << std::fixed << std::setprecision(5);
        for (const auto& step : solution.mpcHorizon) {
            outFile << step.x.x << '\t'
                << step.x.y << '\t'
                << step.x.psi << '\t'
                << step.x.vel << '\t'
                << step.x.cte << '\t'
                << step.x.epsi << '\t'
                << step.u.a << '\t'
                << step.u.delta << std::endl;
        }
    }
}