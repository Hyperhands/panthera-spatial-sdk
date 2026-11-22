#include "panthera/gello.hpp"
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace panthera;
using namespace pybind11::literals;

void bind_gello(py::module_& m) {
    py::class_<SerialPortInfo>(m, "SerialPortInfo")
        .def_readonly("port_name", &SerialPortInfo::port_name)
        .def_readonly("description", &SerialPortInfo::description)
        .def_readonly("vendor_id", &SerialPortInfo::vendor_id)
        .def_readonly("product_id", &SerialPortInfo::product_id)
        .def_readonly("serial_number", &SerialPortInfo::serial_number)
        .def_readonly("manufacturer", &SerialPortInfo::manufacturer);
    m.def("list_serial_ports", &list_serial_ports, py::call_guard<py::gil_scoped_release>());

    py::class_<GelloConfig>(m, "GelloConfig")
        .def(py::init<>())
        .def_readwrite("port", &GelloConfig::port)
        .def_readwrite("baud", &GelloConfig::baud)
        .def_readwrite("group_mask", &GelloConfig::group_mask)
        .def_readwrite("servo_timeout_ms", &GelloConfig::servo_timeout_ms)
        .def_readwrite("reconnect_interval_ms", &GelloConfig::reconnect_interval_ms);

    py::class_<GelloServoSample>(m, "GelloServoSample")
        .def_readonly("id", &GelloServoSample::id)
        .def_readonly("raw", &GelloServoSample::raw)
        .def_readonly("ticks", &GelloServoSample::ticks)
        .def_readonly("velocity", &GelloServoSample::velocity)
        .def_readonly("age_s", &GelloServoSample::age_s)
        .def_readonly("valid", &GelloServoSample::valid);

    py::class_<GelloSnapshot>(m, "GelloSnapshot")
        .def_readonly("connected", &GelloSnapshot::connected)
        .def_readonly("scans", &GelloSnapshot::scans)
        .def_readonly("replies", &GelloSnapshot::replies)
        .def_readonly("timeouts", &GelloSnapshot::timeouts)
        .def_readonly("bad", &GelloSnapshot::bad)
        .def_readonly("jumps", &GelloSnapshot::jumps)
        .def_readonly("scan_hz", &GelloSnapshot::scan_hz)
        .def_readonly("servos", &GelloSnapshot::servos);

    py::class_<GelloReader>(m, "GelloReader", py::release_gil_before_calling_cpp_dtor{})
        .def(py::init<GelloConfig>(), "config"_a, py::call_guard<py::gil_scoped_release>())
        .def(py::init([](std::string port, std::uint32_t baud, std::uint8_t group_mask,
                         int servo_timeout_ms, int reconnect_interval_ms) {
                 GelloConfig cfg;
                 cfg.port = std::move(port);
                 cfg.baud = baud;
                 cfg.group_mask = group_mask;
                 cfg.servo_timeout_ms = servo_timeout_ms;
                 cfg.reconnect_interval_ms = reconnect_interval_ms;
                 py::gil_scoped_release release;
                 return std::make_unique<GelloReader>(std::move(cfg));
             }),
             "port"_a, "baud"_a = 1000000, "group_mask"_a = 0x03, "servo_timeout_ms"_a = 5,
             "reconnect_interval_ms"_a = 500)
        .def("poll", &GelloReader::poll, py::call_guard<py::gil_scoped_release>())
        .def("is_open", &GelloReader::is_open)
        .def("close", &GelloReader::close, py::call_guard<py::gil_scoped_release>())
        .def(
            "__enter__", [](GelloReader& r) -> GelloReader& { return r; },
            py::return_value_policy::reference_internal)
        .def("__exit__", [](GelloReader& r, py::object, py::object, py::object) {
            py::gil_scoped_release release;
            r.close();
        });
}
