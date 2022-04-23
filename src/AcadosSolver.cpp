#include "mpc_local_planner/AcadosSolver.h"

#include <ros/ros.h>

namespace mpc {

AcadosSolver::AcadosSolver(const State &state) {
    init();
    setInitGuess(state);
    return;
}

AcadosSolver::~AcadosSolver() {
    freeAllocated();
    return;
}

void AcadosSolver::reInit(const State &state) {
    freeAllocated();
    init();
    setInitGuess(state);
    return;
}

void AcadosSolver::setInitCondition(const State &state) {
    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;

    auto x0 = state.toArray();
    assert(NBX0 == x0.size());

    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "lbx", &x0[0]);
    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "ubx", &x0[0]);
}

void AcadosSolver::setParams(const Params &params) {
    auto p = params.toArray();
    assert(p.size() == NP);

    for (int ii = 0; ii <= dims_->N; ii++) {
        bicycle_model_acados_update_params(capsule_, ii, &p[0], NP);
    }
}

MPCReturn AcadosSolver::solve(const State &state, const Params &params) {
    auto start = std::chrono::high_resolution_clock::now();
    int N = dims_->N;

    setInitCondition(state);
    setParams(params);

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
        reInit(state);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    MPCReturn ret;
    ret.mpcHorizon.resize(N);
    for (int i = 0; i < N; i++) {
        State state(&xtraj[NX * i], NX);
        Input input(&utraj[NU * i], NU);
        ret.mpcHorizon.at(i) = OptVariables{state, input};
    }
    ret.u0 = ret.mpcHorizon.at(0).u;
    ret.cost = -1;
    ret.success = status == ACADOS_SUCCESS;
    ret.computeTime = duration.count();
    return ret;
}

void AcadosSolver::setInitGuess(const State &state) {

    auto x_init = state.toArray();
    assert(x_init.size() == NX);
    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;

    // initialize solution
    for (int i = 0; i <= dims_->N; i++) {
        ocp_nlp_out_set(config_, dims_, out_, i, "x", &x_init[0]);
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