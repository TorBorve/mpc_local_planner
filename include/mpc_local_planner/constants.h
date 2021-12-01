#pragma once

// Parameters for mpc

#define MPC_N 20 // number of steps in mpc

#define MPC_dt 0.1 // time increment in between steps

// Constants for car

#define MPC_MAX_STEERING_ANGLE 0.57 // maxium steering angle on the wheels in radians

#define MPC_MIN_STEERING_ANGLE -MPC_MAX_STEERING_ANGLE // minimum steering angle on the wheels in radians

#define MPC_MAX_STEERING_ROTATION_SPEED 800.0 / 17.3 * M_PI / 180.0 // maximum ratation speed on the wheels in [rad / s]

// should add LF variable