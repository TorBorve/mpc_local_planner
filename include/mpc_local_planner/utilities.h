#pragma once

#include "mpc_local_planner/MPC.h"

#include <nav_msgs/Path.h>

namespace mpc{
    geometry_msgs::Pose toMsg(const State& state);
    
    nav_msgs::Path getPathMsg(const MPCReturn& solution);

}