#ifndef MPC_ACADOS_SOLVER_H_
#define MPC_ACADOS_SOLVER_H_

#include <ros/ros.h>

#include "mpc_local_planner/types.h"

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
// #include "acados_solver_bicycle_model.h"

// #define NX BICYCLE_MODEL_NX
// #define NZ BICYCLE_MODEL_NZ
// #define NU BICYCLE_MODEL_NU
// #define NP BICYCLE_MODEL_NP
// #define NBX BICYCLE_MODEL_NBX
// #define NBX0 BICYCLE_MODEL_NBX0
// #define NBU BICYCLE_MODEL_NBU
// #define NSBX BICYCLE_MODEL_NSBX
// #define NSBU BICYCLE_MODEL_NSBU
// #define NSH BICYCLE_MODEL_NSH
// #define NSG BICYCLE_MODEL_NSG
// #define NSPHI BICYCLE_MODEL_NSPHI
// #define NSHN BICYCLE_MODEL_NSHN
// #define NSGN BICYCLE_MODEL_NSGN
// #define NSPHIN BICYCLE_MODEL_NSPHIN
// #define NSBXN BICYCLE_MODEL_NSBXN
// #define NS BICYCLE_MODEL_NS
// #define NSN BICYCLE_MODEL_NSN
// #define NG BICYCLE_MODEL_NG
// #define NBXN BICYCLE_MODEL_NBXN
// #define NGN BICYCLE_MODEL_NGN
// #define NY0 BICYCLE_MODEL_NY0
// #define NY BICYCLE_MODEL_NY
// #define NYN BICYCLE_MODEL_NYN
// #define NH BICYCLE_MODEL_NH
// #define NPHI BICYCLE_MODEL_NPHI
// #define NHN BICYCLE_MODEL_NHN
// #define NPHIN BICYCLE_MODEL_NPHIN
// #define NR BICYCLE_MODEL_NR

namespace mpc {
namespace Acados {

/// @brief Class for acados solver. Using functions and more for the generated c code.
class Solver {
   public:
    /// @brief consturctor used to allocated memory and initialize
    /// @param[in] state current state of the car.
    // Solver(const State &state);
    Solver() = default;

    /// @brief destructor used to free allocated memory
    ~Solver() = default;

    /// @brief reinitalize solver.
    /// @param[in] state current state. Used in init.
    void reInit(const State &state);

    /// @brief set constraints for the inital state
    /// @param[in] state the current state
    void setInitCondition(const State &state);

    /// @brief set parameters used by solver. In this case it is the coefficients for the interpolated third degree polynomial and pitch of the car.
    /// @param[in] params the parameters for the solver
    virtual void setParams(const Params &params) = 0;

    /// @brief set the inital guess for states and inputs in solver. Here we set them to the current state and u = 0
    /// @param[in] state the current state of the car
    void setInitGuess(const State &state);

    /// @brief solve function for solving NMPC
    /// @param[in] state current state.
    /// @param[in] params parmeters for solver
    /// @return the optimal solution as MPCReturn type.
    MPCReturn solve(const State &state, const Params &params);

   protected:
    virtual int acadosSolve() = 0;
    /// @brief initalize class. Allocates the needed memory.
    virtual void init() = 0;

    /// @brief free allocated memory in class.
    virtual void freeAllocated() = 0;

    // bicycle_model_capusel* = capsule_

    /// @brief config from capsule
    ocp_nlp_config *config_;

    /// @brief dimensions of nlp
    ocp_nlp_dims *dims_;

    /// @brief nlp in
    ocp_nlp_in *in_;

    /// @brief nlp out
    ocp_nlp_out *out_;

    /// @brief nlp_solver
    ocp_nlp_solver *solver_;

    /// @brief options pointer
    void *opts_;

    size_t nx_, nu_;
};
}  // namespace Acados
}  // namespace mpc

#endif