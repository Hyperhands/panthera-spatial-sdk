#include "panthera/controller.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace panthera;
using namespace pybind11::literals;
void bind_telemetry(py::module_& m);

namespace {
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
template <int N> Eigen::Matrix<double, N, 1> vector(const Array& a) {
    if (a.ndim() != 1 || a.shape(0) != N)
        throw py::value_error("Unexpected vector shape");
    Eigen::Matrix<double, N, 1> v = Eigen::Map<const Eigen::Matrix<double, N, 1>>(a.data());
    if (!v.allFinite())
        throw py::value_error("Expected finite vector");
    return v;
}
void check_interrupt() {
    py::gil_scoped_acquire gil;
    if (PyErr_CheckSignals() < 0)
        throw py::error_already_set();
}
} // namespace
PYBIND11_MODULE(_core, m) {
    bind_telemetry(m);
    py::enum_<Side>(m, "Side").value("left", Side::left).value("right", Side::right);
    py::enum_<ExpertAck>(m, "ExpertAck")
        .value("IUnderstandNoSafety", ExpertAck::IUnderstandNoSafety);
    py::class_<MotionBounds>(m, "MotionBounds")
        .def(py::init(&default_bounds))
        .def_readwrite("lower", &MotionBounds::lower)
        .def_readwrite("upper", &MotionBounds::upper)
        .def_readwrite("max_speed", &MotionBounds::max_speed)
        .def_readwrite("max_acceleration", &MotionBounds::max_acceleration)
        .def_readwrite("max_lead", &MotionBounds::max_lead);
    m.def("default_bounds", &default_bounds);
    py::class_<ControlConfig>(m, "ControlConfig")
        .def(py::init<>())
        .def_readwrite("joints", &ControlConfig::joints)
        .def_readwrite("task_weights", &ControlConfig::task_weights)
        .def_readwrite("pose_gain", &ControlConfig::pose_gain)
        .def_readwrite("joint_gain", &ControlConfig::joint_gain)
        .def_readwrite("regularization", &ControlConfig::regularization)
        .def_readwrite("smoothing", &ControlConfig::smoothing)
        .def_readwrite("posture_weight", &ControlConfig::posture_weight)
        .def_readwrite("translation_speed", &ControlConfig::translation_speed)
        .def_readwrite("rotation_speed", &ControlConfig::rotation_speed);
    py::class_<ArmGeometry>(m, "ArmGeometry")
        .def_property_readonly(
            "tcp", [](const ArmGeometry& g) { return Eigen::Matrix4d(g.tcp().matrix()); })
        .def_property_readonly("frames",
                               [](const ArmGeometry& g) {
                                   std::array<Eigen::Matrix4d, 9> frames;
                                   for (size_t i = 0; i < frames.size(); ++i)
                                       frames[i] = g.frames[i].matrix();
                                   return frames;
                               })
        .def_readonly("jacobian", &ArmGeometry::jacobian)
        .def_readonly("gravity", &ArmGeometry::gravity);
    m.def("arm_geometry", [](Side s, const Array& q) {
        return arm_geometry(static_cast<unsigned>(s), vector<6>(q));
    });
    py::class_<CollisionState>(m, "CollisionState")
        .def_readonly("closest", &CollisionState::closest)
        .def_readonly("inter_arm", &CollisionState::inter_arm)
        .def_readonly("hard_margin", &CollisionState::hard_margin)
        .def_readonly("pair", &CollisionState::pair);
    m.def("collision_state", [](const Array& measured) {
        auto q = vector<12>(measured);
        CollisionState c;
        collision_state({arm_geometry(0, q.head<6>()), arm_geometry(1, q.tail<6>())}, c);
        return c;
    });
    py::class_<RpcTiming>(m, "RpcTiming")
        .def_readonly("completed", &RpcTiming::completed)
        .def_readonly("retransmissions", &RpcTiming::retransmissions)
        .def_property_readonly("last_us", [](const RpcTiming& r) { return r.last.count(); })
        .def_property_readonly("maximum_us", [](const RpcTiming& r) { return r.maximum.count(); });
    py::class_<ControlState>(m, "ControlState")
        .def_readonly("enabled", &ControlState::enabled)
        .def_readonly("frozen", &ControlState::frozen)
        .def_readonly("expert", &ControlState::expert)
        .def_readonly("error", &ControlState::error)
        .def_readonly("consecutive_failures", &ControlState::consecutive_failures)
        .def_readonly("closest_pair", &ControlState::closest_pair)
        .def_readonly("clearance", &ControlState::clearance)
        .def_readonly("hard_margin", &ControlState::hard_margin)
        .def_readonly("measured", &ControlState::measured)
        .def_readonly("reference", &ControlState::reference)
        .def_readonly("velocity", &ControlState::velocity)
        .def_readonly("boards", &ControlState::boards)
        .def_readonly("rpc", &ControlState::rpc)
        .def_readonly("cycles", &ControlState::cycles)
        .def_readonly("missed_cycles", &ControlState::missed_cycles)
        .def_property_readonly("qp_time",
                               [](const ControlState& s) { return s.qp_time.count() * 1e-9; })
        .def_property_readonly("tcp", [](const ControlState& s) {
            return std::array<Eigen::Matrix4d, 2>{s.tcp[0].matrix(), s.tcp[1].matrix()};
        });
    py::class_<DualArm>(m, "DualArm", py::release_gil_before_calling_cpp_dtor{})
        .def(py::init<BoardAddress, BoardAddress, ControlConfig>(), "left"_a = BoardAddress::left(),
             "right"_a = BoardAddress::right(), "config"_a = ControlConfig{})
        .def(
            "enable",
            [](DualArm& arm, int timeout_ms) {
                py::gil_scoped_release release;
                arm.enable(std::chrono::milliseconds(timeout_ms), check_interrupt);
            },
            "timeout_ms"_a = 30000)
        .def("set_joint_target",
             [](DualArm& arm, Side s, const Array& q) { arm.setJointTarget(s, vector<6>(q)); })
        .def("set_twist_target",
             [](DualArm& arm, Side s, const Array& v) { arm.setTwistTarget(s, vector<6>(v)); })
        .def(
            "set_pose_target",
            [](DualArm& arm, Side s, const Array& a, const Array& ff) {
                if (a.ndim() != 2 || a.shape(0) != 4 || a.shape(1) != 4)
                    throw py::value_error("Expected a 4x4 pose");
                Eigen::Isometry3d pose;
                pose.matrix() =
                    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(a.data());
                arm.setPoseTarget(s, pose, vector<6>(ff));
            },
            "side"_a, "pose"_a, "feedforward"_a = Six::Zero().eval())
        .def(
            "set_hand_target",
            [](DualArm& arm, Side s, const Array& a, std::uint16_t duration) {
                Joints q;
                Eigen::Map<Eigen::Matrix<float, 6, 1>>(q.data()) = vector<6>(a).cast<float>();
                arm.setHandTarget(s, q, duration);
            },
            "side"_a, "angles"_a, "duration_ms"_a)
        .def("state", &DualArm::state)
        .def("soft_stop", &DualArm::softStop, py::call_guard<py::gil_scoped_release>())
        .def("enter_expert_mode", &DualArm::enterExpertMode,
             py::call_guard<py::gil_scoped_release>())
        .def("send_mit", &DualArm::sendMit, py::call_guard<py::gil_scoped_release>())
        .def("local_ports", &DualArm::local_ports)
        .def(
            "__enter__", [](DualArm& a) -> DualArm& { return a; },
            py::return_value_policy::reference_internal)
        .def("__exit__", [](DualArm& a, py::object, py::object, py::object) {
            py::gil_scoped_release release;
            a.softStop();
        });
}
