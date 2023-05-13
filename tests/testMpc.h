#pragma once

#include "mpc_local_planner/MPC.h"
#include "mpc_local_planner/utilities.h"
#include <ros/ros.h>
#include <thread>

namespace mpc
{
    void testMpc()
    {
        auto track = mpc::getTestTrack();
        mpc::OptVariables initialOptVars{State{41, 0, M_PI_2, 5.0, 0, 0}, Input{0, 0}};
        constexpr size_t N = 10;
        constexpr double dt = 0.15;
        constexpr double loopHz = 30;
        constexpr double wheelbase = 2.65;
        const Bound steeringAngle{-0.57, 0.57};
        constexpr double maxSteeringRotationSpeed = 0.8;

        mpc::OptVariables optVars = initialOptVars;
        mpc::MPC MPC{track, N, dt, steeringAngle, maxSteeringRotationSpeed, wheelbase};
        auto refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{100};
        std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for rviz
        double totalTime = 0;
        size_t iterations = 0;
        while (ros::ok())
        {
            auto solution = MPC.solve(optVars);

            MPC.model(optVars, solution.u0, 1.0 / loopHz);

            std::cout << "Time: " << solution.computeTime << " [ms]\n";
            totalTime += solution.computeTime;
            iterations++;
            std::cout << "Avg. time: " << static_cast<int>(totalTime / iterations) << " [ms]\n";
            std::cout << "Vel: " << optVars.x.vel << std::endl;

            std::this_thread::sleep_until(refresh);
            refresh = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds{(int)(1000 * 1.0 / loopHz)};
        }
    }
}