#ifndef MPC_BEZIERCURVE_H_
#define MPC_BEZIERCURVE_H_

#include "mpc_local_planner/types.h"

namespace mpc {

/// @brief class for representing and calculating bezier curves
class BezierCurve {
   public:
    /// @brief constructor for bezier curves
    /// @param[in] p1 the start point
    /// @param[in] angle1 the angle of the tangent at p1
    /// @param[in] p2 the end point
    /// @param[in] angle2 the angle of the tangent at p2
    /// @param[in] frac fraction for placing points along tangengt. Should be in the range [0, 0.5]
    /// most likley. Small values give a straighter line, larger gives a more curved line.
    BezierCurve(const Point &p1, double angle1, const Point &p2, double angle2, double frac);

    /// @brief constructor with list of points defining a bezier curve
    BezierCurve(std::initializer_list<Point> list);

    /// @brief calculate a point on the bezier curve
    /// @param[in] points the points defining a bezier curve
    /// @param[in] t how far along the curve we want to get the point. t = [0, 1]
    /// @return the point at that t value
    static Point calc(std::vector<Point> points, double t);

    /// @brief calculate a point on the curve
    /// @param[in] t how far along the curve we want to get the point. t = [0, 1]
    Point calc(double t) const;

    /// @brief linear interpolate. Get a line between to point and return the point at t
    /// @param[in] p1 the start point
    /// @param[in] p2 the end point
    /// @param[in] t ow far along the curve we want to get the point. t = [0, 1]
    static Point lerp(const Point &p1, const Point &p2, double t);

    /// @brief the points defining the bezier curve
    std::vector<Point> points;
};
}  // namespace mpc

#endif