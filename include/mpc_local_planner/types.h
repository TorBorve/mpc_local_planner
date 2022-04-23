#ifndef MPC_TYPES_H_
#define MPC_TYPES_H_

#include <eigen3/Eigen/Core>
#include <vector>

namespace mpc {

/// @brief State struct for car
struct State {
    /// @brief default constructor
    State() = default;

    /// @brief constructor taking in arguments
    /// @param[in] x the x position
    /// @param[in] y the y position
    /// @param[in] psi the yaw angle of car
    /// @param[in] vel the current velocity
    /// @param[in] delta the current steering angle.
    /// @param[in] throttle the current throttle value.
    State(double x, double y, double psi, double vel, double delta, double throttle) : x{x}, y{y}, psi{psi}, vel{vel}, delta{delta}, throttle{throttle} {
    }

    /// @brief constructor with array containig values
    /// @param[in] arr array containg state variables
    State(const std::array<double, 6> &arr) : State{&arr[0], 6} {
    }

    /// @brief constructor using double pointer
    /// @param[in] arr pointer to start of array.
    /// @param[in] size size of array.
    State(double const *arr, size_t size) {
        assert(size == 6);
        *this = State{arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]};
    }

    /// @brief convert state to array containg state varibles
    /// @return array with state varibles
    std::array<double, 6> toArray() const {
        return std::array<double, 6>{x, y, psi, vel, delta, throttle};
    }

    /// @brief x position
    double x;

    /// @brief y position
    double y;

    /// @brief yaw angle
    double psi;

    /// @brief velocity
    double vel;

    /// @brief steering angle.
    double delta;

    /// @brief throttle value
    double throttle;
};

/// @brief Input struct for car. Inputs is what command that are sendt to the car.
struct Input {
    /// @brief default constructor
    Input() = default;

    /// @brief constructor with values.
    /// @param[in] deltaDot derivative of the throttle
    /// @param[in] throttleDot derivative of the steering angle
    Input(double deltaDot, double throttleDot) : deltaDot{deltaDot}, throttleDot{throttleDot} {
    }

    Input(const std::array<double, 2> &arr) : Input{&arr[0], 2} {
    }

    Input(double const *arr, size_t size) {
        assert(size == 2);
        *this = Input{arr[0], arr[1]};
    }

    std::array<double, 2> toArray() const {
        return std::array<double, 2>{deltaDot, throttleDot};
    }

    /// @brief derivative of the steering angle
    double deltaDot;

    /// @brief derivative of the throttle value
    double throttleDot;
};

/// @brief struct for optimal variables used in solver
struct OptVariables {
    /// @brief default constructor
    OptVariables() = default;

    /// @brief constructor with variables
    /// @param[in] x state variable
    /// @param[in] u input variable
    OptVariables(const State &x, const Input &u) : x{x}, u{u} {
    }

    /// @brief the state of the car
    State x;

    /// @brief the inputs sendt to car.
    Input u;
};

/// @brief struct for mpc solution
struct MPCReturn {
    /// @brief defualt constructor
    MPCReturn() = default;

    /// @brief constructor with variables
    /// @param[in] u0 first input to be sendt to car.
    /// @param[in] mpcHorizon the calculated optimal variables from mpc. size = N
    /// @param[in] computeTime the time the solver used in [ms]
    /// @param[in] cost the cost of the mpc horizon.
    /// @param[in] success boolean for indicating if the solver managed to solve the problem.
    MPCReturn(const Input &u0, const std::vector<OptVariables> &mpcHorizon,
              double computeTime, double cost, bool success) : u0{u0}, mpcHorizon{mpcHorizon}, computeTime{computeTime}, cost{cost}, success{success} {
    }

    /// @brief constructor with variables
    /// @param[in] mpcHorizon the calculated optimal variables from mpc. size = N
    /// @param[in] computeTime the time the solver used in [ms]
    /// @param[in] cost the cost of the mpc horizon.
    /// @param[in] success boolean for indicating if the solver managed to solve the problem.
    MPCReturn(const std::vector<OptVariables> &mpcHorizon,
              double computeTime, double cost, bool success) : MPCReturn{mpcHorizon.at(0).u, mpcHorizon, computeTime, cost, success} {
    }

    /// @brief the first input. u0 = mpcHorizon[0].u;
    Input u0;

    /// @brief the calculated optimal variables from mpc.
    std::vector<OptVariables> mpcHorizon;

    /// @brief time used to solve problem in [ms]
    double computeTime;

    /// @brief cost of the optimal solution
    double cost;

    /// @brief boolean for indication if the solver manged to solve the problem
    bool success;
};

/// @brief struct for point (x, y)
struct Point {
    /// @brief default constructor
    Point() = default;

    /// @brief constructor with values
    /// @param[in] x x value of point
    /// @param[in] y y value of point
    Point(double x, double y) : x{x}, y{y} {
    }

    /// @brief the x value
    double x;

    /// @brief the y value
    double y;
};

/// @brief stuct for params used in solver
struct Params {
    Params() = default;

    /// @brief constructor for params
    /// @param[in] polyCoeffs tho coefficient for the 3rd deg. polynomial used to represent the road.
    /// @param[in] pitch the pitch of the car. Indicates if the car is going downhill or uphill.
    Params(const Eigen::Vector4d &polyCoeffs, double pitch) : polyCoeffs{polyCoeffs}, pitch{pitch} {}

    /// @brief constructor using array.
    /// @param[in] arr array to params.
    Params(const std::array<double, 5> &arr) : polyCoeffs{arr[0], arr[1], arr[2], arr[3]}, pitch{arr[4]} {
    }

    /// @brief convert params to array.
    /// @return std::array<double, 5> containg the params
    std::array<double, 5> toArray() const {
        return std::array<double, 5>{polyCoeffs[0], polyCoeffs[1], polyCoeffs[2], polyCoeffs[3], pitch};
    }

    /// @brief the coefficients for the polynomoial defining the track.
    Eigen::Vector4d polyCoeffs;

    /// @brief pitch of the car.
    double pitch;
};
}  // namespace mpc

#endif