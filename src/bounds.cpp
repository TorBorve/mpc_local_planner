#include "mpc_local_planner/bounds.h"

namespace mpc {

Bound::Bound(double lower, double upper) : lower{lower}, upper{upper} {}

Bound Bound::noBound() { return Bound{-1e19, 1e19}; }

Bound Bound::zeroBound() { return Bound{0, 0}; }

bool Bound::operator==(const Bound &rhs) { return upper == rhs.upper && lower == rhs.lower; }

std::vector<double> getUpper(const BoundVector &vec) {
    std::vector<double> upper(vec.size());
    for (unsigned int i = 0; i < vec.size(); i++) {
        upper[i] = vec[i].upper;
    }
    return upper;
}

std::vector<double> getLower(const BoundVector &vec) {
    std::vector<double> lower(vec.size());
    for (unsigned int i = 0; i < vec.size(); i++) {
        lower[i] = vec[i].lower;
    }
    return lower;
}
}  // namespace mpc