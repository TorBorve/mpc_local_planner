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
    /// @param[in] polyCoeffs tho coefficient for the 3rd deg. polynomial used to represent the road.
    /// @param[in] pitch the pitch of the car. Indicates if the car is going downhill or uphill.
    PointStabParams(const Point &pRef, double psiRef, double pitch) : pRef{pRef}, psiRef{psiRef}, pitch{pitch} {}

    /// @brief constructor using vector.
    /// @param[in] vec vector with params.
    PointStabParams(const std::vector<double> &vec) {
        assert(size == vec.size());
        pRef = Point{vec[0], vec[1]};
        psiRef = vec[2];
        pitch = vec[3];
    }

    /// @brief convert params to array.
    /// @return std::array<double, 5> containg the params
    std::vector<double> toVec() const override {
        return std::vector<double>{pRef.x, pRef.y, psiRef, pitch};
    }

    /// @brief the coefficients for the polynomoial defining the track.
    Point pRef;

    double psiRef;

    /// @brief pitch of the car.
    double pitch;

    const size_t size = 4;
};

class PointStab : public Solver {
   public:
    PointStab(const State &state);
    ~PointStab();
    void setParams(const Params &params) override;

   private:
    int acadosSolve() override;
    void init() override;
    void freeAllocated() override;

    point_stab_solver_capsule *capsule_;
};
}  // namespace Acados
}  // namespace mpc

#endif