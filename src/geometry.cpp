#include "panthera/geometry.hpp"
#include "panthera_left.h"
#include "panthera_right.h"
#include <algorithm>
#include <stdexcept>
namespace panthera {
namespace {
using V = Eigen::Vector3d;
using Map = Eigen::Map<const V>;
struct Capsule {
    int side, frame;
    double a[3], b[3], r;
};
struct Witness {
    double normal[3], offset, center[3], axis[3][3];
};
struct Group {
    int first, count;
};
struct Pair {
    int a, b;
    double hard, safe, capsule_hard;
    int first, count;
};
#include "geometry.inc"
struct Segment {
    V a, b;
};
Segment closest_points(const Segment& a, const Segment& b) {
    const V u = a.b - a.a, v = b.b - b.a, w = a.a - b.a;
    const double uu = u.dot(u), vv = v.dot(v), uv = u.dot(v), uw = u.dot(w), vw = v.dot(w);
    const double det = u.cross(v).squaredNorm();
    const auto unit = [](double s) { return std::clamp(s, 0., 1.); };
    double s = det > 1e-20 ? unit((uv * vw - vv * uw) / det) : 0;
    double t = vv > 1e-20 ? unit((uv * s + vw) / vv) : 0;
    if (uu > 1e-20)
        s = unit((uv * t - uw) / uu);
    return {a.a + s * u, b.a + t * v};
}
void project(JointVector& row, const ArmGeometry& arm, const Capsule& cap, const V& at,
             const V& n) {
    for (int j = 0; j < std::min(cap.frame, 6); ++j)
        row[6 * cap.side + j] +=
            n.dot(arm.jacobian.block<3, 1>(3, j).cross(at - arm.frames[j + 1].translation()));
}
struct Bound {
    double distance;
    V n, point_a, point_b;
};
Bound certificate(const Pair& pair, const Eigen::Isometry3d& ta, const Eigen::Isometry3d& tb,
                  Bound best) {
    for (int k = 0; k < pair.count; ++k) {
        auto g = groups[pair.first + k];
        Bound weakest{std::numeric_limits<double>::infinity(), V::Zero(), V::Zero(), V::Zero()};
        for (int l = 0; l < g.count; ++l) {
            const auto& w = witnesses[g.first + l];
            const V n = ta.linear() * Map(w.normal), nb = tb.linear().transpose() * n;
            V corner = Map(w.center);
            for (auto& axis : w.axis)
                corner -= std::copysign(1., nb.dot(Map(axis))) * Map(axis);
            V point = tb * corner;
            double gap = n.dot(point - ta.translation()) - w.offset;
            if (gap < weakest.distance)
                weakest = {gap, -n, point, point};
            if (gap <= best.distance)
                break;
        }
        if (weakest.distance > best.distance)
            best = weakest;
    }
    return best;
}
} // namespace
ArmGeometry arm_geometry(unsigned side, const Six& q) {
    if (side > 1 || !q.allFinite())
        throw std::invalid_argument("Invalid arm configuration");
    ArmGeometry out;
    Eigen::Matrix<double, 3, 36> poses;
    const double* arg[] = {q.data()};
    double* result[] = {poses.data(), out.jacobian.data(), out.gravity.data()};
    (side ? panthera_right : panthera_left)(arg, result, nullptr, nullptr, 0);
    for (size_t i = 0; i < out.frames.size(); ++i) {
        out.frames[i].setIdentity();
        out.frames[i].matrix().topRows<3>() = poses.middleCols<4>(4 * i);
    }
    return out;
}
MotionBounds default_bounds() {
    MotionBounds b;
    for (int i = 0; i < 12; ++i) {
        b.lower[i] = lower[i % 6];
        b.upper[i] = upper[i % 6];
    }
    b.max_speed.setConstant(.8);
    b.max_acceleration.setConstant(4.);
    b.max_lead.setConstant(.05);
    return b;
}
void collision_state(const std::array<ArmGeometry, 2>& arms, CollisionState& state) {
    std::array<Segment, std::size(capsules)> segments;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& c = capsules[i];
        auto& t = arms[c.side].frames[c.frame];
        segments[i] = {t * Map(c.a), t * Map(c.b)};
    }
    state.rows.clear();
    state.inter_arm = state.closest = state.hard_margin = std::numeric_limits<double>::infinity();
    state.pair = -1;
    for (size_t k = 0; k < std::size(pairs); ++k) {
        auto& p = pairs[k];
        auto& a = capsules[p.a];
        auto& b = capsules[p.b];
        auto nearest = closest_points(segments[p.a], segments[p.b]);
        V delta = nearest.a - nearest.b;
        double length = delta.norm(), raw = length - a.r - b.r;
        Bound bound{raw, delta / std::max(length, 1e-15), nearest.a, nearest.b};
        if (p.count && raw < .1)
            bound =
                certificate(p, arms[a.side].frames[a.frame], arms[b.side].frames[b.frame], bound);
        if (a.side != b.side)
            state.inter_arm = std::min(state.inter_arm, bound.distance);
        if (bound.distance < state.closest) {
            state.closest = bound.distance;
            state.pair = k;
        }
        state.hard_margin =
            std::min({state.hard_margin, bound.distance - p.hard, raw - p.capsule_hard});
        auto add_row = [&](const Bound& contact, double safe, double influence, int id) {
            if (contact.distance >= influence)
                return;
            JointVector gradient = JointVector::Zero();
            project(gradient, arms[a.side], a, contact.point_a, contact.n);
            project(gradient, arms[b.side], b, contact.point_b, -contact.n);
            const double margin = contact.distance - safe;
            const bool same_arm = a.side == b.side;
            const double lower = -.5 * margin / (influence - safe);
            state.rows.push_back(
                {gradient, same_arm ? std::min(0., lower) : lower, margin, id, same_arm});
        };
        add_row(bound, p.safe, .1, 2 * k);
        // The candidate must also respect the measured broad-phase hard limit.
        // Give that independent limit its own velocity damper when a mesh
        // certificate supplies a tighter positive physical separation.
        if (p.count && bound.distance > raw + 1e-10) {
            Bound capsule{raw, delta / std::max(length, 1e-15), nearest.a, nearest.b};
            add_row(capsule, p.capsule_hard + .0005, p.capsule_hard + .08, 2 * k + 1);
        }
    }
}
} // namespace panthera
