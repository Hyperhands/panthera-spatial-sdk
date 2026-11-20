#include <Eigen/Core>
#include <acados_c/ocp_qp_interface.h>

#include "value_init.h"

#include <array>
#include <cstdint>

int main() {
    arm_command_value_t command{};
    init_arm_command(&command);
    init_stream_header(&command.header);
    command.header.schema_revision = 7;
    command.header.source_node = 10;
    command.header.target_node = 11;
    command.header.source_boot_id = 1234;
    command.header.target_boot_id = 5678;
    command.header.sequence = 1;
    command.control_lease = 17;
    command.policy_revision = 3;
    command.joint_mask = 63;
    const Eigen::Matrix<float, 6, 1> positions =
        Eigen::Matrix<float, 6, 1>::LinSpaced(6, 0.0f, 0.5f);
    for (int joint = 0; joint < positions.size(); ++joint) {
        command.mit[5 * joint] = positions[joint];
    }
    std::array<std::uint8_t, 512> packet{};
    size_t length{};
    arm_command_value_t decoded{};
    if (arm_command_value_encode(&command, packet.data(), packet.size(), &length) != 0 ||
        arm_command_value_decode(packet.data(), length, &decoded) != 0 ||
        decoded.header.target_boot_id != command.header.target_boot_id ||
        decoded.joint_mask != command.joint_mask || decoded.mit[25] != positions[5]) {
        return 1;
    }

    ocp_qp_solver_plan_t plan{};
    plan.qp_solver = FULL_CONDENSING_HPIPM;
    auto* config = ocp_qp_xcond_solver_config_create(plan);
    if (!config) {
        return 1;
    }
    ocp_qp_xcond_solver_config_free(config);
    return 0;
}
