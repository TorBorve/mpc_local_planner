#ifndef MPC_BOUNDS_H_
#define MPC_BOUNDS_H_

#include <vector>

/// @brief namespace for Fuel Fighter MPC
namespace mpc {

/// @brief Bound class for variable bounds and constraint bounds for solver
struct Bound {
    /// @brief default constructor
    Bound() = default;

    /// @brief constructor with upper and lower bounds
    /// @param[in] lower minimum allowed value
    /// @param[in] upper maximum allowed value
    Bound(double lower, double upper);

    /// @return A Bound object with lower = -"inf" and upper = "inf"
    static Bound noBound();

    /// @return A Bound object with lower = upper = 0
    static Bound zeroBound();

    /// @brief check if bounds are equal
    /// @return true if lower1 == lower2 and upper1 == upper2
    bool operator==(const Bound &rhs);

    /// @brief lowest value allowed
    double lower;

    /// @brief highest value allowed
    double upper;
};

/// @brief alias for vector containing bounds
using BoundVector = std::vector<Bound>;

/// @brief get the upper values from BoundVector
/// @return std::vector with upper bounds
std::vector<double> getUpper(const BoundVector &vec);

/// @brief get the lower values from BoundVector
/// @return std::vector with lower bounds
std::vector<double> getLower(const BoundVector &vec);
}  // namespace mpc

#endif