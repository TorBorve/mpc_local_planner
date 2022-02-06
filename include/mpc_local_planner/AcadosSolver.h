// #pragma once
#ifndef MPC_ACADOS_SOLVER_H_
#define MPC_ACADOS_SOLVER_H_

#include "mpc_local_planner/types.h"

#include <ros/ros.h>

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_bicycle_model.h"

#define NX BICYCLE_MODEL_NX
#define NZ BICYCLE_MODEL_NZ
#define NU BICYCLE_MODEL_NU
#define NP BICYCLE_MODEL_NP
#define NBX BICYCLE_MODEL_NBX
#define NBX0 BICYCLE_MODEL_NBX0
#define NBU BICYCLE_MODEL_NBU
#define NSBX BICYCLE_MODEL_NSBX
#define NSBU BICYCLE_MODEL_NSBU
#define NSH BICYCLE_MODEL_NSH
#define NSG BICYCLE_MODEL_NSG
#define NSPHI BICYCLE_MODEL_NSPHI
#define NSHN BICYCLE_MODEL_NSHN
#define NSGN BICYCLE_MODEL_NSGN
#define NSPHIN BICYCLE_MODEL_NSPHIN
#define NSBXN BICYCLE_MODEL_NSBXN
#define NS BICYCLE_MODEL_NS
#define NSN BICYCLE_MODEL_NSN
#define NG BICYCLE_MODEL_NG
#define NBXN BICYCLE_MODEL_NBXN
#define NGN BICYCLE_MODEL_NGN
#define NY0 BICYCLE_MODEL_NY0
#define NY BICYCLE_MODEL_NY
#define NYN BICYCLE_MODEL_NYN
#define NH BICYCLE_MODEL_NH
#define NPHI BICYCLE_MODEL_NPHI
#define NHN BICYCLE_MODEL_NHN
#define NPHIN BICYCLE_MODEL_NPHIN
#define NR BICYCLE_MODEL_NR

namespace mpc
{
    /// @brief Class for acados solver. Using functions and more for the generated c code.
    class AcadosSolver
    {
    public:
        /// @brief consturctor used to allocated memory and initialize
        /// @param[in] optVars current state and inputs
        AcadosSolver(const OptVariables &optVars);

        /// @brief destructor used to free allocated memory
        ~AcadosSolver();

        /// @brief reinitalize solver.
        /// @param[in] optVars current state and inputs. Used in init.
        void reInit(const OptVariables &optVars);

        /// @brief set constraints for the inital state
        /// @param[in] optVars the current state and inputs
        void setInitCondition(const OptVariables &optVars);

        /// @brief set parameters used by solver. In this case it is the coefficients for the interpolated third degree polynomial
        /// @param[in] coeffs the coeffs for polynomial defining track
        void setParams(const Eigen::Vector4d &coeffs);

        /// @brief set the initalguess for states and inputs in solver. Here we set them to the current state and u = 0
        /// @param[in] optVars the current positon and inputs.
        void setInitGuess(const OptVariables &optVars);

        /// @brief solve function for solving NMPC
        /// @param[in] optVars current state and inputs.
        /// @param[in] coeffs coefficients for polynomial defining track
        /// @return the optimal solution as MPCReturn type.
        MPCReturn solve(const OptVariables &optVars, const Eigen::Vector4d &coeffs);

    private:
        /// @brief initalize class
        /// @param[in] optVars current state and inputs
        void init();

        /// @brief free allocated memory in class
        void freeAllocated();

        /// @brief solver capsule from acados
        bicycle_model_solver_capsule *capsule_;

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
    };
}

#endif