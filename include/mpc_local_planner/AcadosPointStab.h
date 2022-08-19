#ifndef MPC_ACADOS_POINT_H_
#define MPC_ACADOS_POINT_H_

#include "acados_solver_point_stab.h"
#include "mpc_local_planner/AcadosSolver.h"
#include "mpc_local_planner/types.h"

namespace mpc {
namespace Acados {

struct PointStabParams : public Params {
    PointStabParams() = default;

    /// @brief constructor for params
    /// @param[in] polyCoeffs tho coefficient for the 3rd deg. polynomial used to represent the
    /// road.
    /// @param[in] pitch the pitch of the car. Indicates if the car is going downhill or uphill.
    PointStabParams(const Point &pRef, double psiRef, double pitch)
        : pRef{pRef}, psiRef{psiRef}, pitch{pitch} {}

    /// @brief constructor using vector.
    /// @param[in] vec vector with params.
    PointStabParams(const std::vector<double> &vec) {
        assert(size == vec.size());
        pRef = Point{vec[0], vec[1]};
        psiRef = vec[2];
        pitch = vec[3];
    }

    /// @brief convert params to vector.
    /// @return vector containg the params
    std::vector<double> toVec() const override {
        return std::vector<double>{pRef.x, pRef.y, psiRef, pitch};
    }

    /// @brief the reference point where we want the car to drive to.
    Point pRef;

    /// @brief the reference yaw of the car at the location we want to go to.
    double psiRef;

    /// @brief pitch of the car.
    double pitch;

    /// @brief the total number of params.
    const size_t size = 4;
};

/// @brief solver for point stabilization
class PointStab : public Solver {
   public:
    /// @brief constructor taking in the state of the vehicle
    PointStab(const State &state);

    /// @brief destructor
    ~PointStab();

    /// @brief set parameters for the solver
    /// @param[in] params params class for point stab.
    void setParams(const Params &params) override;

   private:
    /// @brief solve function used. Need to set params and constraints before this.
    int acadosSolve() override;

    /// @brief init function. Allocates memory for the pointers
    void init() override;

    /// @brief deletes the allocated memory.
    void freeAllocated() override;

    /// @brief capsule for point stab solver
    point_stab_solver_capsule *capsule_;
};
}  // namespace Acados
}  // namespace mpc

#endif