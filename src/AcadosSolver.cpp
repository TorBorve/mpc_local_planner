#include "mpc_local_planner/AcadosSolver.h"

#include <rclcpp/rclcpp.hpp>

namespace mpc {
namespace Acados {

void Solver::reInit(const State &state) {
    freeAllocated();
    init();
    setInitGuess(state);
    return;
}

void Solver::setInitCondition(const State &state) {
    // initial condition
    auto x0 = state.toArray();
    std::vector<int> idxbx0(x0.size());
    for (unsigned int i = 0; i < x0.size(); i++) {
        idxbx0[i] = i;
    }

    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "idxbx", &idxbx0[0]);
    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "lbx", &x0[0]);
    ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "ubx", &x0[0]);
}

MPCReturn Solver::solve(const State &state, const Params &params) {
    auto start = std::chrono::high_resolution_clock::now();
    int N = dims_->N;

    setInitCondition(state);
    setParams(params);

    // prepare evaluation
    int NTIMINGS = 1;

    std::vector<double> xtraj(nx_ * (N + 1));
    std::vector<double> utraj(nx_ * (N));

    // solve ocp in loop
    int rti_phase = 0;
    int status;

    for (int ii = 0; ii < NTIMINGS; ii++) {
        ocp_nlp_solver_opts_set(config_, opts_, "rti_phase", &rti_phase);
        status = acadosSolve();
    }

    // get the solution
    for (int ii = 0; ii <= dims_->N; ii++)
        ocp_nlp_out_get(config_, dims_, out_, ii, "x", &xtraj[ii * nx_]);
    for (int ii = 0; ii < dims_->N; ii++)
        ocp_nlp_out_get(config_, dims_, out_, ii, "u", &utraj[ii * nu_]);

    if (status != ACADOS_SUCCESS) {
        std::cout << "ERROS: acados_solve() failed with status " << status << std::endl;
        reInit(state);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    MPCReturn ret;
    ret.mpcHorizon.resize(N);
    for (int i = 0; i < N; i++) {
        State state(&xtraj[nx_ * i], nx_);
        Input input(&utraj[nu_ * i], nu_);
        ret.mpcHorizon.at(i) = OptVariables{state, input};
    }
    ret.u0 = ret.mpcHorizon.at(0).u;
    ret.cost = -1;
    ret.success = status == ACADOS_SUCCESS;
    ret.computeTime = duration.count();
    return ret;
}

void Solver::setInitGuess(const State &state) {
    auto x_init = state.toArray();
    assert(x_init.size() == nx_);
    // initial value for control input
    std::vector<double> u0(nu_);
    for (unsigned int i = 0; i < nu_; i++) {
        u0[0] = 0;
    }

    // initialize solution
    for (int i = 0; i <= dims_->N; i++) {
        ocp_nlp_out_set(config_, dims_, out_, i, "x", &x_init[0]);
        ocp_nlp_out_set(config_, dims_, out_, i, "u", &u0[0]);
    }
}
}  // namespace Acados
}  // namespace mpc