#include "mpc_local_planner/AcadosPointStab.h"

namespace mpc {
namespace Acados {

PointStab::PointStab(const State &state) {
    init();
    setInitGuess(state);
    return;
}

PointStab::~PointStab() {
    freeAllocated();
    return;
}

void PointStab::setParams(const Params &params) {
    auto p = params.toVec();
    assert(p.size() == POINT_STAB_NP);

    for (int ii = 0; ii <= dims_->N; ii++) {
        point_stab_acados_update_params(capsule_, ii, &p[0], POINT_STAB_NP);
    }
}

int PointStab::acadosSolve() { return point_stab_acados_solve(capsule_); }

void PointStab::init() {
    nx_ = POINT_STAB_NX;
    nu_ = POINT_STAB_NU;
    capsule_ = point_stab_acados_create_capsule();
    int N = POINT_STAB_N;
    // allocate the array and fill it accordingly
    double *new_time_steps = NULL;
    int status = point_stab_acados_create_with_discretization(capsule_, N, new_time_steps);
    config_ = point_stab_acados_get_nlp_config(capsule_);
    dims_ = point_stab_acados_get_nlp_dims(capsule_);
    in_ = point_stab_acados_get_nlp_in(capsule_);
    out_ = point_stab_acados_get_nlp_out(capsule_);
    solver_ = point_stab_acados_get_nlp_solver(capsule_);
    opts_ = point_stab_acados_get_nlp_opts(capsule_);
    if (status) {
        std::stringstream error;
        error << "point_stab_acados_create() returned status: " << status << ". Exiting.";
        std::cout << "Error: " << error.str() << std::endl;
        throw std::runtime_error{error.str()};
    }
    return;
}

void PointStab::freeAllocated() {
    // free solver
    int status = point_stab_acados_free(capsule_);
    if (status) {
        std::cout << "Error: " << "point_stab_acados_free() returned status " << status << std::endl;
    }
    // free solver capsule
    status = point_stab_acados_free_capsule(capsule_);
    if (status) {
        std::cout << "Error: " << "point_stab_acados_free_capsule() returned status " << status << std::endl;
    }
    return;
}
}  // namespace Acados
}  // namespace mpc