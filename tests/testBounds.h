#pragma once
#include "mpc_local_planner/bounds.h"

#include <assert.h>

namespace mpc
{
    void testBounds()
    {
        Bound b1{1, 2};
        assert(b1.lower == 1 && b1.upper == 2 && "testing constructor");
        b1 = Bound::noBound();
        assert(b1.lower < -1e15 && b1.upper > 1e15 && "test Bound::noBound");
        b1 = Bound::zeroBound();
        assert(b1.lower == 0 && b1.upper == 0 && "test Bound::zeroBound");
        std::vector<Bound> bVec{Bound::noBound(), Bound::zeroBound(), Bound{1, 2}};
        assert(bVec[1] == Bound::zeroBound() && "test intializer list for BoundVector");
        bVec.push_back(b1);
        auto upper = getUpper(bVec);
        auto lower = getLower(bVec);
        assert(bVec.size() == upper.size() && bVec.size() == lower.size() && "test correct size");
        for (int i = 0; i < bVec.size(); i++)
        {
            assert(bVec[i].lower == lower[i] && bVec[i].upper == upper[i] && "test getUpper and getLower");
        }
        return;
    }
}