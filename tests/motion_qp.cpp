#include "panthera/motion_qp.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace panthera;
void check(bool value, const char* msg) {
    if (!value)
        throw std::runtime_error(msg);
}
void check_offset_speed_boxes(MotionQp& qp) {
    const JointVector zero = JointVector::Zero(), one = JointVector::Ones();
    MotionBounds bounds{-2 * one, 2 * one, .25 * one, 4 * one, .05 * one};
    const JointMatrix h = 1.00111 * JointMatrix::Identity();
    // Replay ticks 2520 and 2546. The left J3 saturates while the right J3 optimum moves inside its
    // speed box.
    auto lift = [&](double left, double right, double goal_left, double goal_right) {
        JointVector previous = zero, desired = zero;
        previous[2] = left;
        previous[8] = right;
        desired[2] = goal_left;
        desired[8] = goal_right;
        const auto result = qp.solve(zero, zero, previous, h, -h * desired, bounds, {});
        JointVector expected = desired;
        expected[2] = std::min(.25, left + .008);
        check(result.success && (result.velocity - expected).lpNorm<Eigen::Infinity>() < 1e-6,
              "offset J3 speed boxes on ready lift");
    };
    lift(.25, .248182845, .340713058, .244214505);
    lift(.199999989, .199999756, .299109188, .202612471);

    // At the folded-pose transition neither arm can satisfy its closing-speed row
    // within one acceleration step. Return control to the servo so it can freeze.
    JointVector closing = zero;
    closing[2] = .24805;
    closing[7] = .24803;
    closing[8] = .24807;
    std::array<CollisionRow, 2> folded{};
    for (int side = 0; side < 2; ++side) {
        auto& row = folded[side];
        row.gradient.setZero();
        row.gradient.segment<3>(6 * side + 2) << (side ? -.05535 : -.05534), .02451, .01395;
        row.clearance_margin = side ? -.000172 : -.000170;
    }
    check(!qp.solve(zero, zero, closing, h, zero, bounds, folded).success,
          "incompatible folded-pose collision rows accepted");

    bounds.max_speed.setConstant(.8);
    const JointVector normal = JointVector::Unit(2) - JointVector::Unit(8);
    const std::array<CollisionRow, 1> rows{{{normal, 0., 0.}}};
    for (int k = -16; k <= 16; ++k) {
        JointVector previous = zero;
        previous[2] = previous[8] = k * .05;
        JointVector expected = previous;
        expected[2] = expected[8] = std::clamp(previous[2], -.796, .796);
        // The projection onto u[left J3] >= u[right J3] is exactly expected.
        const JointVector desired = expected - .04 * normal;
        const auto result = qp.solve(zero, zero, previous, h, -h * desired, bounds, rows);
        check(result.success && (result.velocity - expected).lpNorm<Eigen::Infinity>() < 1e-6,
              "active collision row in offset speed box");
    }
}

int main() try {
    MotionQp qp;
    check_offset_speed_boxes(qp);
    auto z = JointVector::Zero().eval(), one = JointVector::Ones().eval();
    MotionBounds bounds{-one, one, one, 1000 * one, .05 * one};
    auto q = z;
    q[0] = .999;
    auto ref = q;
    ref[0] = 1.0;
    auto h = JointMatrix::Identity().eval();
    auto lin = -one;
    auto bound = qp.solve(q, ref, z, h, lin, bounds, {});
    check(bound.success && bound.position[0] <= 1.00000001, "lagging joint at upper limit");
    std::vector<CollisionRow> rows(17, {one, -4, .01});
    rows.back() = {-JointVector::Unit(4), -.1, .02};
    auto result = qp.solve(z, z, z, h, lin, bounds, rows);
    check(result.success && result.solves == 2 && result.velocity[4] <= .1000001,
          "omitted closing pair");
    std::vector<double> times;
    for (int i = 0; i < 1000; ++i) {
        result = qp.solve(z, z, z, h, lin, bounds, {});
        if (i > 10)
            times.push_back(result.solve_time.count() / 1000.);
    }
    std::sort(times.begin(), times.end());
    std::cout << "HPIPM p99 " << times[times.size() * 99 / 100] << " us\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
