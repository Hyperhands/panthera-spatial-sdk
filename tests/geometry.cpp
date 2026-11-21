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
    // Independent triangle-mesh query confirms link2/link5 intersect here.
    left << 0., -.5, 0., 0., 2., 0.;
    collision_state({arm_geometry(0, left * deg), arm_geometry(1, right * deg)}, c);
    if (c.hard_margin >= 0.)
        throw std::runtime_error("Intersecting link2/link5 accepted");
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
