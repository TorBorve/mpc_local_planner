#include "mpc_local_planner/AcadosSolver.h"

#include <ros/ros.h>

namespace mpc {

AcadosSolver::AcadosSolver(const OptVariables &optVars) {
    init();
    setInitGuess(optVars);
    return;
}

AcadosSolver::~AcadosSolver() {
    freeAllocated();
    return;
}

void AcadosSolver::reInit(const OptVariables &optVars) {
    freeAllocated();
    init();
    setInitGuess(optVars);
    return;
}

void AcadosSolver::setInitCondition(const OptVariables &optVars) {
    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = optVars.x.x;
    ubx0[0] = optVars.x.x;
    lbx0[1] = optVars.x.y;
    ubx0[1] = optVars.x.y;
    lbx0[2] = optVars.x.psi;
    ubx0[2] = optVars.x.psi;
    lbx0[3] = optVars.x.vel;
    ubx0[3] = optVars.x.vel;
    lbx0[4] = optVars.u.delta;
    ubx0[4] = optVars.u.delta;
    lbx0[5] = optVars.u.throttle;
    ubx0[5] = optVars.u.throttle;

    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "ubx", ubx0);
}

void AcadosSolver::setParams(const Eigen::Vector4d &coeffs) {
    double p[NP];
    p[0] = coeffs[0];
    p[1] = coeffs[1];
    p[2] = coeffs[2];
    p[3] = coeffs[3];

    for (int ii = 0; ii <= dims_->N; ii++) {
        bicycle_model_acados_update_params(capsule_, ii, p, NP);
    }
}

MPCReturn AcadosSolver::solve(const OptVariables &optVars, const Eigen::Vector4d &coeffs) {
    OptVariables optVarsCopy{optVars};
    static double prevThrottle = 0.0;
    optVarsCopy.u.throttle = prevThrottle;
    int N = dims_->N;
    auto start = std::chrono::high_resolution_clock::now();

    setInitCondition(optVarsCopy);
    setParams(coeffs);

    // prepare evaluation
    int NTIMINGS = 1;

    double xtraj[NX * (N + 1)];
    double utraj[NU * N];

    // solve ocp in loop
    int rti_phase = 0;
    int status;

    for (int ii = 0; ii < NTIMINGS; ii++) {
        ocp_nlp_solver_opts_set(config_, opts_, "rti_phase", &rti_phase);
        status = bicycle_model_acados_solve(capsule_);
    }

    // get the solution
    for (int ii = 0; ii <= dims_->N; ii++)
        ocp_nlp_out_get(config_, dims_, out_, ii, "x", &xtraj[ii * NX]);
    for (int ii = 0; ii < dims_->N; ii++)
        ocp_nlp_out_get(config_, dims_, out_, ii, "u", &utraj[ii * NU]);

    if (status != ACADOS_SUCCESS) {
        ROS_ERROR("bicycle_model_acados_solve() failed with status %d.\n", status);
        reInit(optVarsCopy);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    MPCReturn ret;
    ret.mpcHorizon.resize(N);
    ret.u0 = Input{xtraj[5 + NX], xtraj[4 + NX]};
    prevThrottle = ret.u0.throttle;
    for (int i = 0; i < N; i++) {
        State state{xtraj[0 + NX * i], xtraj[1 + NX * i], xtraj[2 + NX * i], xtraj[3 + NX * i], 0, 0};
        Input input{xtraj[5 + NX * i], xtraj[4 + NX * i]};
        ret.mpcHorizon.at(i) = OptVariables{state, input};
    }
    ret.cost = -1;
    ret.success = status == ACADOS_SUCCESS;
    ret.computeTime = duration.count();
    return ret;
}

void AcadosSolver::setInitGuess(const OptVariables &optVars) {
    // initialization for state values
    double x_init[NX];
    x_init[0] = optVars.x.x;
    x_init[1] = optVars.x.y;
    x_init[2] = optVars.x.psi;
    x_init[3] = optVars.x.vel;
    x_init[4] = optVars.u.delta;
    x_init[5] = optVars.u.throttle;

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;

    // initialize solution
    for (int i = 0; i <= dims_->N; i++) {
        ocp_nlp_out_set(config_, dims_, out_, i, "x", x_init);
        ocp_nlp_out_set(config_, dims_, out_, i, "u", u0);
    }
}

void AcadosSolver::init() {
    capsule_ = bicycle_model_acados_create_capsule();
    int N = BICYCLE_MODEL_N;
    // allocate the array and fill it accordingly
    double *new_time_steps = NULL;
    int status = bicycle_model_acados_create_with_discretization(capsule_, N, new_time_steps);
    config_ = bicycle_model_acados_get_nlp_config(capsule_);
    dims_ = bicycle_model_acados_get_nlp_dims(capsule_);
    in_ = bicycle_model_acados_get_nlp_in(capsule_);
    out_ = bicycle_model_acados_get_nlp_out(capsule_);
    solver_ = bicycle_model_acados_get_nlp_solver(capsule_);
    opts_ = bicycle_model_acados_get_nlp_opts(capsule_);
    if (status) {
        std::stringstream error;
        error << "bicycle_model_acados_create() returned status: " << status << ". Exiting.";
        ROS_ERROR_STREAM(error.str());
        throw std::runtime_error{error.str()};
    }
    return;
}

void AcadosSolver::freeAllocated() {
    // free solver
    int status = bicycle_model_acados_free(capsule_);
    if (status) {
        ROS_ERROR("bicycle_model_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = bicycle_model_acados_free_capsule(capsule_);
    if (status) {
        ROS_ERROR("bicycle_model_acados_free_capsule() returned status %d. \n", status);
    }
    return;
}
}  // namespace mpc