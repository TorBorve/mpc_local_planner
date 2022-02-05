#include "mpc_local_planner/solverAcados.h"
#include <ros/ros.h>

namespace mpc
{
    // AcadosSolver::AcadosSolver(const OptVariables &optVars)
    // {
    //     init(optVars);
    //     return;
    // }

    // void AcadosSolver::init(const OptVariables &optVars)
    // {
    //     capsule = bicycle_model_acados_create_capsule();
    //     int N = BICYCLE_MODEL_N;
    //     // allocate the array and fill it accordingly
    //     double *new_time_steps = NULL;
    //     int status = bicycle_model_acados_create_with_discretization(capsule, N, new_time_steps);
    //     config = bicycle_model_acados_get_nlp_config(capsule);
    //     dims = bicycle_model_acados_get_nlp_dims(capsule);
    //     in = bicycle_model_acados_get_nlp_in(capsule);
    //     out = bicycle_model_acados_get_nlp_out(capsule);
    //     solver = bicycle_model_acados_get_nlp_solver(capsule);
    //     opts = bicycle_model_acados_get_nlp_opts(capsule);
    //     if (status)
    //     {
    //         printf("bicycle_model_acados_create() returned status %d. Exiting.\n", status);
    //         exit(1);
    //     }

    //     // initialization for state values
    //     double x_init[NX];
    //     x_init[0] = optVars.x.x;
    //     x_init[1] = optVars.x.y;
    //     x_init[2] = optVars.x.psi;
    //     x_init[3] = optVars.x.vel;
    //     x_init[4] = optVars.u.delta;

    //     // initial value for control input
    //     double u0[NU];
    //     u0[0] = 0.0;
    //     u0[1] = 0.0;

    //     // initialize solution
    //     for (int i = 0; i <= dims->N; i++)
    //     {
    //         ocp_nlp_out_set(config, dims, out, i, "x", x_init);
    //         ocp_nlp_out_set(config, dims, out, i, "u", u0);
    //     }
    //     return;        
    // }

    // void AcadosSolver::freeAllocated()
    // {
    //     // free solver
    //     int status = bicycle_model_acados_free(capsule);
    //     if (status) {
    //         ROS_ERROR("bicycle_model_acados_free() returned status %d. \n", status);
    //     }
    //     // free solver capsule
    //     status = bicycle_model_acados_free_capsule(capsule);
    //     if (status) {
    //         ROS_ERROR("bicycle_model_acados_free_capsule() returned status %d. \n", status);
    //     }
    //     return;
    // }
}