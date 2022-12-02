#include "mpc_local_planner/AcadosPathTracking.h"

namespace mpc {
namespace Acados {

PathTracking::PathTracking(const State &state) {
    init();
    setInitGuess(state);
    return;
}

PathTracking::~PathTracking() {
    freeAllocated();
    return;
}

void PathTracking::setParams(const Params &params) {
    auto p = params.toVec();
    assert(p.size() == PATH_TRACKING_NP);

    for (int ii = 0; ii <= dims_->N; ii++) {
        path_tracking_acados_update_params(capsule_, ii, &p[0], PATH_TRACKING_NP);
    }
}

int PathTracking::acadosSolve() { return path_tracking_acados_solve(capsule_); }

void PathTracking::init() {
    nx_ = PATH_TRACKING_NX;
    nu_ = PATH_TRACKING_NU;
    capsule_ = path_tracking_acados_create_capsule();
    int N = PATH_TRACKING_N;
    // allocate the array and fill it accordingly
    double *new_time_steps = NULL;
    int status = path_tracking_acados_create_with_discretization(capsule_, N, new_time_steps);
    config_ = path_tracking_acados_get_nlp_config(capsule_);
    dims_ = path_tracking_acados_get_nlp_dims(capsule_);
    in_ = path_tracking_acados_get_nlp_in(capsule_);
    out_ = path_tracking_acados_get_nlp_out(capsule_);
    solver_ = path_tracking_acados_get_nlp_solver(capsule_);
    opts_ = path_tracking_acados_get_nlp_opts(capsule_);
    if (status) {
        std::stringstream error;
        error << "path_tracking_acados_create() returned status: " << status << ". Exiting.";
        std::cout << "Error: " << error.str() << std::endl;
        throw std::runtime_error{error.str()};
    }
    return;
}

void PathTracking::freeAllocated() {
    // free solver
    int status = path_tracking_acados_free(capsule_);
    if (status) {
        std::cout << "Error: " << "point_stab_acados_free() returned status " << status << std::endl;
    }
    // free solver capsule
    status = path_tracking_acados_free_capsule(capsule_);
    if (status) {
        std::cout << "Error: " << "point_stab_acados_free_capsule() returned status " << status << std::endl;

    }
    return;
}
}  // namespace Acados
}  // namespace mpc