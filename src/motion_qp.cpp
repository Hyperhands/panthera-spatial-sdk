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
    enum class BoxStatus { solved, infeasible, failed };
    BoxStatus solve_box(QpStep& result, const JointVector& reference, const JointVector& lo,
                        const JointVector& hi, const JointVector& vlo, const JointVector& vhi,
                        const JointMatrix& hessian, const JointVector& linear,
                        std::span<const CollisionRow> rows, int budget, Clock::time_point deadline,
                        bool projection = false);
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
    const auto deadline = Clock::now() + std::chrono::milliseconds(2);
    const int budget = std::min(3, max_solves);
    if (!measured.allFinite() || !reference.allFinite() || !previous.allFinite() ||
        !hessian.allFinite() || !linear.allFinite() || !limits.lower.allFinite() ||
        !limits.upper.allFinite() || !limits.max_speed.allFinite() ||
        !limits.max_lead.allFinite() || !limits.max_acceleration.allFinite() ||
        (limits.max_acceleration.array() <= 0.).any())
        return result;
    const JointVector lo = limits.lower.cwiseMax(measured - limits.max_lead);
    const JointVector hi = limits.upper.cwiseMin(measured + limits.max_lead);
    // Position, tracking lead, joint-limit damping and speed always remain hard.
    const JointVector safe_lo = (-limits.max_speed)
                                    .cwiseMax((8 * (limits.lower - measured)).cwiseMin(0))
                                    .cwiseMax((lo - reference) / control_dt);
    const JointVector safe_hi =
        limits.max_speed.cwiseMin((8 * (limits.upper - measured)).cwiseMax(0))
            .cwiseMin((hi - reference) / control_dt);
    if ((lo.array() > hi.array()).any() || !safe_lo.allFinite() || !safe_hi.allFinite() ||
        (safe_lo.array() > safe_hi.array()).any())
        return result;
    for (const auto& row : rows)
        if (!row.gradient.allFinite() || !std::isfinite(row.lower) ||
            !std::isfinite(row.clearance_margin))
            return result;

    const JointVector dv = control_dt * limits.max_acceleration;
    const auto status =
        impl_->solve_box(result, reference, lo, hi, safe_lo.cwiseMax(previous - dv),
                         safe_hi.cwiseMin(previous + dv), hessian, linear, rows, budget, deadline);
    if (status == Impl::BoxStatus::infeasible && result.solves < budget && Clock::now() < deadline)
        impl_->solve_box(result, reference, lo, hi, safe_lo, safe_hi, JointMatrix::Identity(),
                         -previous, rows, budget, deadline, true);
    return result;
}
MotionQp::Impl::BoxStatus MotionQp::Impl::solve_box(QpStep& result, const JointVector& reference,
                                                    const JointVector& lo, const JointVector& hi,
                                                    const JointVector& vlo, const JointVector& vhi,
                                                    const JointMatrix& hessian,
                                                    const JointVector& linear,
                                                    std::span<const CollisionRow> rows, int budget,
                                                    Clock::time_point deadline, bool projection) {
    if ((vlo.array() > vhi.array()).any())
        return BoxStatus::infeasible;
    order.clear();
    order.reserve(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& row = rows[i];
        const JointVector at_lo = row.gradient.cwiseProduct(vlo);
        const JointVector at_hi = row.gradient.cwiseProduct(vhi);
        // No slot is needed if the whole velocity box satisfies the row.
        if (at_lo.cwiseMin(at_hi).sum() >= row.lower - 1e-12)
            continue;
        // If even its best corner violates the row, this box is infeasible.
        if (at_lo.cwiseMax(at_hi).sum() < row.lower - tolerance)
            return BoxStatus::infeasible;
        order.push_back(i);
    }
    // For a stationary joint target with positive diagonal cost, zero velocity is
    // the exact minimizer. Accept it only when it satisfies every bound and row.
    if (linear.isZero(0.) && hessian.isDiagonal(0.) && hessian.diagonal().minCoeff() > 0. &&
        vlo.maxCoeff() <= 0. && vhi.minCoeff() >= 0. &&
        std::all_of(rows.begin(), rows.end(), [](const auto& row) { return row.lower <= 0.; })) {
        result.success = Clock::now() < deadline;
        return result.success ? BoxStatus::solved : BoxStatus::failed;
    }
    set(0, "R", hessian.data());
    set(0, "r", linear.data());
    set(0, "b", reference.data());
    set(0, "lbu", vlo.data());
    set(0, "ubu", vhi.data());
    const size_t used = std::min<size_t>(slots, order.size());
    std::partial_sort(order.begin(), order.begin() + used, order.end(), [&](auto a, auto b) {
        return rows[a].clearance_margin < rows[b].clearance_margin;
    });
    while (result.solves < budget) {
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
        set(0, "lg_mask", mask.data());
        set(0, "D", d.data());
        set(0, "lg", lb.data());
        set(0, "ug", ub.data());
        auto before = Clock::now();
        if (before >= deadline)
            return BoxStatus::failed;
        int status = ocp_qp_solve(solver, in, out);
        result.solve_time += Clock::now() - before;
        ++result.solves;
        if (Clock::now() >= deadline)
            return BoxStatus::failed;
        if (status != 0) {
            if (status != ACADOS_MAXITER && status != ACADOS_MINSTEP)
                return BoxStatus::failed;
            // A nonnegative combination of rows is itself a necessary inequality.
            // Use solver multipliers only as candidate weights; verify the separation
            // on the actual box before relaxing acceleration on a numerical failure.
            std::array<double, 2 * (12 + slots)> lam{};
            ocp_qp_out_get(out, 0, "lam", lam.data());
            JointVector normal = JointVector::Zero();
            double lower = 0., weight_sum = 0.;
            for (size_t i = 0; i < used; ++i) {
                const auto& row = rows[order[i]];
                const double weight =
                    std::max(0., lam[12 + i]) / std::max(row.gradient.norm(), 1e-10);
                normal += weight * row.gradient;
                lower += weight * row.lower;
                weight_sum += weight;
            }
            const double best = normal.cwiseProduct(vlo).cwiseMax(normal.cwiseProduct(vhi)).sum();
            if (std::isfinite(best) && std::isfinite(lower) &&
                lower - best > tolerance * weight_sum)
                return BoxStatus::infeasible;
            return BoxStatus::failed;
        }
        JointVector u, x;
        ocp_qp_out_get(out, 0, "u", u.data());
        ocp_qp_out_get(out, 1, "x", x.data());
        if (!u.allFinite() || !x.allFinite() ||
            (x - reference - control_dt * u).lpNorm<Eigen::Infinity>() > tolerance ||
            (x.array() < lo.array() - tolerance).any() ||
            (x.array() > hi.array() + tolerance).any() ||
            (u.array() < vlo.array() - tolerance).any() ||
            (u.array() > vhi.array() + tolerance).any())
            return BoxStatus::failed;
        if (projection) {
            // Coordinates absent from the selected rows have an exact scalar projection.
            // At a speed bound an IPM can otherwise leave them slightly inside the box.
            for (int j = 0; j < u.size(); ++j) {
                if (d.col(j).isZero(0.)) {
                    u[j] = std::clamp(-linear[j], vlo[j], vhi[j]);
                    x[j] = reference[j] + control_dt * u[j];
                }
            }
        }
        auto slack = [&](size_t i) { return rows[i].gradient.dot(u) - rows[i].lower; };
        if (std::all_of(rows.begin(), rows.end(),
                        [&](const auto& r) { return r.gradient.dot(u) >= r.lower - tolerance; })) {
            if (Clock::now() >= deadline)
                return BoxStatus::failed;
            result.position = x;
            result.velocity = u;
            result.success = true;
            return BoxStatus::solved;
        }
        // Replace loose slots with the most violated constraints, then check all again.
        std::partial_sort(order.begin(), order.begin() + used, order.end(),
                          [&](auto a, auto b) { return slack(a) < slack(b); });
    }
    return BoxStatus::failed;
}
} // namespace panthera
