#include "mpc_local_planner/BezierCurve.h"

#include "mpc_local_planner/utilities.h"

namespace mpc {

BezierCurve::BezierCurve(const Point &p1, double angle1, const Point &p2, double angle2,
                         double frac) {
    double dist = sqrt(util::distSqrd(p1.x - p2.x, p1.y - p2.y));
    Point p1Tangent{p1.x + frac * cos(angle1) * dist, p1.y + frac * sin(angle1) * dist};
    Point p2Tangent{p2.x - frac * cos(angle2) * dist, p2.y - frac * sin(angle2) * dist};
    *this = BezierCurve{p1, p1Tangent, p2Tangent, p2};
}

BezierCurve::BezierCurve(std::initializer_list<Point> list) : points{list} {}

Point BezierCurve::calc(std::vector<Point> points, double t) {
    for (int n = points.size(); n > 0; n--) {
        for (int i = 0; i < n - 1; i++) {
            points[i] = lerp(points[i], points[i + 1], t);
        }
    }
    return points[0];
}

Point BezierCurve::calc(double t) const { return calc(points, t); }

Point BezierCurve::lerp(const Point &p1, const Point &p2, double t) {
    double dx = (p2.x - p1.x) * t;
    double dy = (p2.y - p1.y) * t;
    return Point{p1.x + dx, p1.y + dy};
}

}  // namespace mpc