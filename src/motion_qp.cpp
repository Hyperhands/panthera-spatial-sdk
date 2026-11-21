#include "panthera/motion_qp.hpp"
#include <acados_c/ocp_qp_interface.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vector>
namespace panthera {
namespace {
constexpr int slots = 16;
constexpr double tolerance = 1e-7;
using Clock = std::chrono::steady_clock;
} // namespace
struct MotionQp::Impl {
    ocp_qp_xcond_solver_config* config;
    ocp_qp_xcond_solver_dims* dims;
    ocp_qp_in* in;
    ocp_qp_out* out;
    ocp_qp_xcond_solver_opts* opts;
    ocp_qp_solver* solver;
    std::vector<size_t> order;
    template <class T> void set(int stage, const char* name, const T* values) {
        ocp_qp_in_set(config, in, stage, const_cast<char*>(name), const_cast<T*>(values));
    }
    Impl() {
        ocp_qp_solver_plan_t plan{};
        plan.qp_solver = FULL_CONDENSING_HPIPM;
        config = ocp_qp_xcond_solver_config_create(plan);
        dims = ocp_qp_xcond_solver_dims_create(config, 1);
        auto dimension = [&](int stage, const char* field, int n) {
            ocp_qp_xcond_solver_dims_set(config, dims, stage, field, &n);
        };
        dimension(0, "nu", 12);
        dimension(0, "nbu", 12);
        dimension(0, "ng", slots);
        dimension(1, "nx", 12);
        in = ocp_qp_in_create_from_xcond_dims(dims);
        out = ocp_qp_out_create_from_xcond_dims(dims);
        opts =
            static_cast<ocp_qp_xcond_solver_opts*>(ocp_qp_xcond_solver_opts_create(config, dims));
        // Avoid HPIPM's heuristic slack initialization for narrow, offset speed boxes.
        // Use symmetric slack/multiplier initialization and separate primal/dual step lengths.
        int warm = 0, t0_init = 0, split_step = 1;
        ocp_qp_xcond_solver_opts_set(config, opts, "warm_start", &warm);
        ocp_qp_xcond_solver_opts_set(config, opts, "t0_init", &t0_init);
        ocp_qp_xcond_solver_opts_set(config, opts, "split_step", &split_step);
        solver = ocp_qp_create(config, dims, opts);
        JointMatrix integrator = control_dt * JointMatrix::Identity();
        std::array<int, 12> indices;
        std::iota(indices.begin(), indices.end(), 0);
        set(0, "B", integrator.data());
        set(0, "idxbu", indices.data());
        std::array<double, slots> zeros{};
        set(0, "ug_mask", zeros.data());
    }
    ~Impl() {
        ocp_qp_solver_destroy(solver);
        ocp_qp_xcond_solver_opts_free(opts);
        ocp_qp_out_free(out);
        ocp_qp_in_free(in);
        ocp_qp_xcond_solver_dims_free(dims);
        ocp_qp_xcond_solver_config_free(config);
    }
};
MotionQp::MotionQp() : impl_(std::make_unique<Impl>()) {}
MotionQp::~MotionQp() = default;
QpStep MotionQp::solve(const JointVector& measured, const JointVector& reference,
                       const JointVector& previous, const JointMatrix& hessian,
                       const JointVector& linear, const MotionBounds& limits,
                       std::span<const CollisionRow> rows, int max_solves) {
    QpStep result{reference, JointVector::Zero()};
    const auto start = Clock::now();
    auto& qp = *impl_;
    JointVector lo = limits.lower.cwiseMax(measured - limits.max_lead);
    JointVector hi = limits.upper.cwiseMin(measured + limits.max_lead);
    // Reduce allowable speed over the final 1/8 s of approach to a joint limit.
    JointVector slow = (8 * (limits.lower - measured)).cwiseMin(0);
    JointVector fast = (8 * (limits.upper - measured)).cwiseMax(0);
    JointVector vlo = (-limits.max_speed)
                          .cwiseMax(previous - control_dt * limits.max_acceleration)
                          .cwiseMax(slow);
    JointVector vhi =
        limits.max_speed.cwiseMin(previous + control_dt * limits.max_acceleration).cwiseMin(fast);
    if (!measured.allFinite() || !reference.allFinite() || !previous.allFinite() ||
        !hessian.allFinite() || !linear.allFinite() || !lo.allFinite() || !hi.allFinite() ||
        !vlo.allFinite() || !vhi.allFinite() || (lo.array() > hi.array()).any() ||
        (vlo.array() > vhi.array()).any())
        return result;
    // x1 = reference + dt*u. Translate its bounds into equivalent velocity
    // bounds before condensing, eliminating twelve redundant HPIPM rows.
    vlo = vlo.cwiseMax((lo - reference) / control_dt);
    vhi = vhi.cwiseMin((hi - reference) / control_dt);
    if ((vlo.array() > vhi.array()).any())
        return result;
    qp.set(0, "R", hessian.data());
    qp.set(0, "r", linear.data());
    qp.set(0, "b", reference.data());
    qp.set(0, "lbu", vlo.data());
    qp.set(0, "ubu", vhi.data());
    auto& order = qp.order;
    order.resize(rows.size());
    std::iota(order.begin(), order.end(), 0);

    for (auto& r : rows)
        if (!r.gradient.allFinite() || !std::isfinite(r.lower) ||
            !std::isfinite(r.clearance_margin))
            return result;
    // A row satisfied throughout the velocity box needs no solver slot. This
    // also handles rigid pairs with an identically zero distance derivative.
    std::erase_if(order, [&](auto i) {
        const auto& row = rows[i];
        double worst =
            row.gradient.cwiseProduct(vlo).cwiseMin(row.gradient.cwiseProduct(vhi)).sum();
        return worst >= row.lower - 1e-12;
    });
    const size_t used = std::min<size_t>(slots, order.size());
    std::partial_sort(order.begin(), order.begin() + used, order.end(), [&](auto a, auto b) {
        return rows[a].clearance_margin < rows[b].clearance_margin;
    });
    for (int attempt = 0; attempt < std::min(3, max_solves); ++attempt) {
        Eigen::Matrix<double, slots, 12> d = Eigen::Matrix<double, slots, 12>::Zero();
        Eigen::Matrix<double, slots, 1> lb = Eigen::Matrix<double, slots, 1>::Constant(-10),
                                        ub = Eigen::Matrix<double, slots, 1>::Constant(100);
        for (size_t i = 0; i < used; ++i) {
            const auto& r = rows[order[i]];
            const auto scale = std::max(r.gradient.norm(), 1e-10);
            d.row(i) = r.gradient.transpose() / scale;
            lb[i] = r.lower / scale;
        }
        std::array<double, slots> mask{};
        std::fill_n(mask.begin(), used, 1.);
        qp.set(0, "lg_mask", mask.data());
        qp.set(0, "D", d.data());
        qp.set(0, "lg", lb.data());
        qp.set(0, "ug", ub.data());
        auto before = Clock::now();
        int status = ocp_qp_solve(qp.solver, qp.in, qp.out);
        result.solve_time += Clock::now() - before;
        ++result.solves;
        if (status != 0 || Clock::now() - start > std::chrono::milliseconds(2))
            return result;
        JointVector u, x;
        ocp_qp_out_get(qp.out, 0, "u", u.data());
        ocp_qp_out_get(qp.out, 1, "x", x.data());
        if (!u.allFinite() || !x.allFinite() ||
            (x - reference - control_dt * u).lpNorm<Eigen::Infinity>() > tolerance ||
            (x.array() < lo.array() - tolerance).any() ||
            (x.array() > hi.array() + tolerance).any() ||
            (u.array() < vlo.array() - tolerance).any() ||
            (u.array() > vhi.array() + tolerance).any())
            return result;
        auto slack = [&](size_t i) { return rows[i].gradient.dot(u) - rows[i].lower; };
        if (std::all_of(rows.begin(), rows.end(),
                        [&](const auto& r) { return r.gradient.dot(u) >= r.lower - tolerance; })) {
            result.position = x;
            result.velocity = u;
            result.success = true;
            return result;
        }
        // Replace loose slots with the most violated constraints, then check all again.
        std::partial_sort(order.begin(), order.begin() + used, order.end(),
                          [&](auto a, auto b) { return slack(a) < slack(b); });
    }
    return result;
}
} // namespace panthera
