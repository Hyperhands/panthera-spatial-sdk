#include "panthera/geometry.hpp"
#include <iostream>
#include <stdexcept>
using namespace panthera;
int main() try {
    CollisionState c;
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
