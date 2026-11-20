#include "panthera/motion_qp.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace panthera;
void check(bool value, const char* msg) {
    if (!value)
        throw std::runtime_error(msg);
}
int main() try {
    MotionQp qp;
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
