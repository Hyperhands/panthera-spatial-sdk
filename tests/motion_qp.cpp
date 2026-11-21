#include "panthera/motion_qp.hpp"
#include "servo_clock.hpp"
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
    // within one acceleration step. Both rows must hold during braking.
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
    const auto brake = qp.solve(zero, zero, closing, h, zero, bounds, folded);
    check(brake.success && (brake.velocity - closing).norm() < closing.norm(),
          "folded pose must brake less abruptly than freezing");
    for (const auto& row : folded)
        check(row.gradient.dot(brake.velocity) >= row.lower - 1e-7, "brake violated folded row");

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

void check_safety_projection(MotionQp& qp) {
    const JointVector z = JointVector::Zero(), one = JointVector::Ones();
    MotionBounds bounds{-2 * one, 2 * one, one, 8 * one, .08 * one};
    const auto h = JointMatrix::Identity().eval();
    JointVector previous = .15 * one, normal = z;
    previous[1] = 1.; // An unrelated axis already at its hard speed limit.
    previous.segment<3>(8) << -.67418031, .06654983, .01012964;
    normal.segment<3>(8) << .264726, .0831067, .00842035;
    std::array<CollisionRow, 1> row{{{normal, -.141542, .0280253}}};
    const JointVector projection =
        previous + normal * ((row[0].lower - normal.dot(previous)) / normal.squaredNorm());
    const auto brake = qp.solve(z, z, previous, h, one, bounds, row, 1);
    check(brake.success && brake.solves == 1 && normal.dot(brake.velocity) >= row[0].lower - 1e-7 &&
              (brake.velocity - projection).lpNorm<Eigen::Infinity>() < 1e-7,
          "tick 679 minimum braking projection");

    bounds.max_lead.setConstant(.05);
    bounds.max_acceleration.setConstant(4.);
    JointVector ref = z;
    ref[3] = .04984;
    previous[3] = .25;
    const auto lead = qp.solve(z, ref, previous, h, one, bounds, {});
    JointVector expected = previous;
    expected[3] = .08;
    check(lead.success && (lead.velocity - expected).lpNorm<Eigen::Infinity>() < 1e-7 &&
              lead.position[3] <= .05 + 1e-7,
          "tick 139 lead projection");
    ref[3] = .07;
    check(!qp.solve(z, ref, previous, h, one, bounds, {}).success, "hard lead box is empty");
    for (double sign : {-1., 1.}) {
        JointVector edge = z, outward = z;
        edge[0] = sign * 1.99;
        outward[0] = sign * .6;
        const auto damped = qp.solve(edge, edge, outward, h, one, bounds, {});
        check(damped.success && std::abs(damped.velocity[0] - sign * .08) < 1e-7,
              "braking must preserve the joint-limit damper");
    }
    row[0].lower = 2.;
    check(!qp.solve(z, z, previous, h, one, bounds, row).success, "infeasible hard collision row");
    const std::array<CollisionRow, 2> joint_conflict{
        {{JointVector::Unit(0) + JointVector::Unit(1), .012, 0.},
         {JointVector::Unit(0) - JointVector::Unit(1), .012, 0.}}};
    const auto coupled = qp.solve(z, z, z, h, z, bounds, joint_conflict);
    check(coupled.success && coupled.solves <= 3 && std::abs(coupled.velocity[0] - .012) < 1e-7,
          "jointly infeasible smooth rows");
    check(!qp.solve(z, z, z, h, z, bounds, joint_conflict, 1).success, "exhausted solve budget");
}

int main() try {
    MotionQp qp;
    check_offset_speed_boxes(qp);
    check_safety_projection(qp);
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
    std::array<CollisionRow, 1> hold_row{{{JointVector::Unit(0), 0., 0.}}};
    auto hold = qp.solve(z, z, z, h, z, bounds, hold_row);
    check(hold.success && hold.solves == 0 && hold.position == z && hold.velocity == z,
          "feasible stationary joint hold must be exact");
    hold_row[0].lower = .1;
    hold = qp.solve(z, z, z, h, z, bounds, hold_row);
    check(hold.success && hold.solves > 0 && hold.velocity[0] >= .1 - 1e-7,
          "stationary target must respect outward collision velocity");
    std::vector<CollisionRow> rows(17, {one, -4, .01});
    rows.back() = {-JointVector::Unit(4), -.1, .02};
    auto result = qp.solve(z, z, z, h, lin, bounds, rows);
    check(result.success && result.solves == 2 && result.velocity[4] <= .1000001,
          "omitted closing pair");
    // The solver returns after its deadline. Reject its candidate even when feasible.
    detail::ServoClock::advance_per_read = std::chrono::milliseconds(1);
    auto late = qp.solve(z, z, z, h, lin, bounds, {});
    check(!late.success && late.solves == 1 && late.position == z && late.velocity == z,
          "late QP result was accepted");
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
