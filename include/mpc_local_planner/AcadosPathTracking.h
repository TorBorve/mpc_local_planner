#ifndef MPC_ACADOS_PATH_TRACKING_H_
#define MPC_ACADOS_PATH_TRACKING_H_

#include "mpc_local_planner/AcadosSolver.h"
#include "mpc_local_planner/types.h"

#include "acados_solver_path_tracking.h"

namespace mpc {
namespace Acados {

struct PathTrackingParams : public Params{
    PathTrackingParams() = default;

    /// @brief constructor for params
    /// @param[in] polyCoeffs tho coefficient for the 3rd deg. polynomial used to represent the road.
    /// @param[in] pitch the pitch of the car. Indicates if the car is going downhill or uphill.
    PathTrackingParams(const Eigen::Vector4d &polyCoeffs, double pitch) : polyCoeffs{polyCoeffs}, pitch{pitch} {}

    /// @brief constructor using vector.
    /// @param[in] vec vector with params.
    PathTrackingParams(const std::vector<double> &vec) {
        assert(size == vec.size());
        polyCoeffs = Eigen::Vector4d{vec[0], vec[1], vec[2], vec[3]};
        pitch = vec[4];
    }

    /// @brief convert params to array.
    /// @return std::array<double, 5> containg the params
    std::vector<double> toVec() const override {
        return std::vector<double>{polyCoeffs[0], polyCoeffs[1], polyCoeffs[2], polyCoeffs[3], pitch};
    }

    /// @brief the coefficients for the polynomoial defining the track.
    Eigen::Vector4d polyCoeffs;

    /// @brief pitch of the car.
    double pitch;

    const size_t size = 5;

};

/// @brief Class for acados solver. Using functions and more for the generated c code.
class PathTracking : public Solver {
   public:
    PathTracking(const State &state);
    ~PathTracking();
    /// @brief set parameters used by solver. In this case it is the coefficients for the interpolated third degree polynomial and pitch of the car.
    /// @param[in] params the parameters for the solver
    void setParams(const Params &params) override;

   private:
    int acadosSolve() override;
    /// @brief initalize class. Allocates the needed memory.
    void init() override;

    /// @brief free allocated memory in class.
    void freeAllocated() override;

    /// @brief solver capsule from acados
    path_tracking_solver_capsule *capsule_;
};

}  // namespace Acados
}  // namespace mpc

#endif