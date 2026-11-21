#include "panthera/geometry.hpp"
#include <iostream>
#include <numbers>
#include <stdexcept>
using namespace panthera;
int main() try {
    CollisionState c;
    constexpr double deg = std::numbers::pi / 180.;
    Six left, right;
    left << -.406, -.055, .857, -1.203, -.352, .566;
    right << -.291, -.022, 1.527, -.989, -.280, .478;
    for (double wrist_offset : {-.05, 0., .05}) {
        Six nearby = left;
        nearby[4] += wrist_offset;
        collision_state({arm_geometry(0, nearby * deg), arm_geometry(1, right * deg)}, c);
        if (c.hard_margin <= 0.)
            throw std::runtime_error("False hard stop at measured home pose");
    }
    // Use a well-separated opposite arm so nominal-zero clearance cannot mask asymmetry.
    Six ready;
    ready << 0, 1, 1.3, -.5, 0, 0;
    auto swapped_states = [&](Six degrees) {
        const Six q = degrees * deg;
        std::array<CollisionState, 2> result;
        collision_state({arm_geometry(0, q), arm_geometry(1, ready)}, result[0]);
        collision_state({arm_geometry(0, ready), arm_geometry(1, q)}, result[1]);
        return result;
    };
    for (const Six& sample : {Six{-.406, -.049, .857, -1.203, -.352, .566},
                              Six{-.406, -.055, .857, -1.489, -.423, .571},
                              Six{-.286, -.022, 1.527, -.989, -.286, .483}}) {
        const auto [left_state, right_state] = swapped_states(sample);
        if (std::min(left_state.hard_margin, right_state.hard_margin) <= 0 ||
            left_state.pair != 30 || right_state.pair != 284 ||
            std::abs(left_state.closest - right_state.closest) > 1e-8)
            throw std::runtime_error("Asymmetric or nonpositive margin at measured pose");
    }
    for (const Six& sample : {Six{0., -.5, 0., -2.5, 1., 0.}, Six{0., -.5, 0., -2.5, -1., 0.},
                              Six{0., -.5, 0., 0., 2., 0.}, Six{0., 0., 0., -1., 0., 0.}}) {
        const auto [left_state, right_state] = swapped_states(sample);
        if (std::max(left_state.hard_margin, right_state.hard_margin) >= 0)
            throw std::runtime_error("Intersecting link2/link5 accepted");
    }
    for (int k = 0; k <= 100; ++k) {
        Six q = Six::Zero();
        q[1] = q[2] = k * 0.000872664626;
        collision_state({arm_geometry(0, q), arm_geometry(1, q)}, c);
        if (c.hard_margin <= 0)
            throw std::runtime_error("Hard stop on zero lift at " + std::to_string(k) + " margin " +
                                     std::to_string(c.hard_margin));
    }
    std::cout << "5 degree lift clearance " << c.closest << " m\n";
    Six q;
    q << 0, 1, 1.3, -.5, 0, 0;
    collision_state({arm_geometry(0, q), arm_geometry(1, q)}, c);
    if (c.hard_margin <= 0)
        return 1;
    q << -.030775491, .434116572, .752034128, -.051771853, -.036911413, .132018223;
    collision_state({arm_geometry(0, q), arm_geometry(1, q)}, c);
    if (c.hard_margin <= 0)
        return 1;
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
