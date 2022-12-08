#include <rclcpp/rclcpp.hpp>

#include "mpc_local_planner/RosMpc.h"
#include "mpc_local_planner/utilities.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    mpc::util::initLogger();

    auto mpcPtr = std::make_shared<mpc::RosMpc>();
    mpcPtr->verifyInputs();
    LOG_STREAM("Topics and transfroms are ok!");
    rclcpp::sleep_for(std::chrono::seconds{2});
    rclcpp::Rate rate{std::chrono::milliseconds{33}};
    LOG_STREAM("Starting loop");

    while (rclcpp::ok()) {
        rclcpp::spin_some(mpcPtr);
        auto start = std::chrono::high_resolution_clock::now();
        auto ret = mpcPtr->solve();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        LOG_DEBUG("Total time: %ld[ms]", duration.count());
        LOG_DEBUG("Time diff: %f[ms]", duration.count() - ret.computeTime);
        rate.sleep();
    }
    return 0;
}