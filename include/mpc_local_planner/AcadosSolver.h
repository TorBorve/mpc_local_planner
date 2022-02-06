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
    class AcadosSolver
    {
    public:
        AcadosSolver(const OptVariables &optVars);
        ~AcadosSolver();
        void reInit(const OptVariables &optVars);

        bicycle_model_solver_capsule *capsule;
        ocp_nlp_config *config;
        ocp_nlp_dims *dims;
        ocp_nlp_in *in;
        ocp_nlp_out *out;
        ocp_nlp_solver *solver;
        void *opts;

    private:
        void init(const OptVariables &optVars);
        void freeAllocated();
    };
}

#endif