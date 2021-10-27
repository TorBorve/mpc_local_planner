#pragma once

#include <vector>

namespace mpc {
    
    struct Bound {
        Bound() = default;

        Bound(double lower, double upper);
        
        static Bound noBound();

        static Bound zeroBound();

        bool operator==(const Bound& rhs);

        double lower;
        double upper;
    };

    using BoundVector = std::vector<Bound>;

    std::vector<double> getUpper(const BoundVector& vec);

    std::vector<double> getLower(const BoundVector& vec);
}
