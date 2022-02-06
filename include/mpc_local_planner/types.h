#ifndef MPC_TYPES_H_
#define MPC_TYPES_H_ 

#include <vector>
#include <eigen3/Eigen/Core>

namespace mpc
{

    /// @brief State struct for car
    struct State
    {

        /// @brief default constructor
        State() = default;

        /// @brief constructor taking in arguments
        /// @param[in] x the x position
        /// @param[in] y the y position
        /// @param[in] psi the yaw angle of car
        /// @param[in] vel the current velocity
        /// @param[in] cte cross track error. Distance from trajectory to car.
        /// @param[in] epsi yaw error. Diffrence between car yaw and trajectory yaw.
        State(double x, double y, double psi, double vel, double cte, double epsi) : x{x}, y{y}, psi{psi}, vel{vel}, cte{cte}, epsi{epsi}
        {
        }

        /// @brief constructor with array containig values
        /// @param[in] arr array containg state variables
        State(const std::array<double, 6> &arr) : State{arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]}
        {
        }

        /// @brief convert state to array containg state varibles
        /// @return array with state varibles
        std::array<double, 6> toArray() const
        {
            return std::array<double, 6>{x, y, psi, vel, cte, epsi};
        }

        /// @brief x position
        double x;

        /// @brief y position
        double y;

        /// @brief yaw angle
        double psi;

        /// @brief velocity
        double vel;

        /// @brief cross track error. Distance from car to trajectory
        double cte;

        /// @brief yaw error. diffrence between car and trajectory yaw.
        double epsi;
    };

    /// @brief Input struct fro car. Inputs is what command that are sendt to the car.
    struct Input
    {

        /// @brief default constructor
        Input() = default;

        /// @brief constructor with values.
        /// @param[in] a acceleration
        /// @param[in] delta steering angle
        Input(double a, double delta) : a{a}, delta{delta}
        {
        }

        /// @brief acceleration for car
        double a;

        /// @brief steering angle on wheels
        double delta;
    };

    /// @brief struct for optimal variables used in solver
    struct OptVariables
    {

        /// @brief default constructor
        OptVariables() = default;

        /// @brief constructor with variables
        /// @param[in] x state variable
        /// @param[in] u input variable
        OptVariables(const State &x, const Input &u) : x{x}, u{u}
        {
        }

        /// @brief the state of the car
        State x;

        /// @brief the inputs sendt to car.
        Input u;
    };

    /// @brief struct for mpc solution
    struct MPCReturn
    {

        /// @brief defualt constructor
        MPCReturn() = default;

        /// @brief constructor with variables
        /// @param[in] u0 first input to be sendt to car.
        /// @param[in] mpcHorizon the calculated optimal variables from mpc. size = N
        /// @param[in] computeTime the time the solver used in [ms]
        /// @param[in] cost the cost of the mpc horizon.
        /// @param[in] success boolean for indicating if the solver managed to solve the problem.
        MPCReturn(const Input &u0, const std::vector<OptVariables> &mpcHorizon,
                  double computeTime, double cost, bool success) : u0{u0}, mpcHorizon{mpcHorizon}, computeTime{computeTime},
                                                                   cost{cost}, success{success}
        {
        }

        /// @brief constructor with variables
        /// @param[in] mpcHorizon the calculated optimal variables from mpc. size = N
        /// @param[in] computeTime the time the solver used in [ms]
        /// @param[in] cost the cost of the mpc horizon.
        /// @param[in] success boolean for indicating if the solver managed to solve the problem.
        MPCReturn(const std::vector<OptVariables> &mpcHorizon,
                  double computeTime, double cost, bool success) : MPCReturn{mpcHorizon.at(0).u, mpcHorizon, computeTime, cost, success}
        {
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
    struct Point
    {

        /// @brief default constructor
        Point() = default;

        /// @brief constructor with values
        /// @param[in] x x value of point
        /// @param[in] y y value of point
        Point(double x, double y) : x{x}, y{y}
        {
        }

        /// @brief the x value
        double x;

        /// @brief the y value
        double y;
    };
}

#endif