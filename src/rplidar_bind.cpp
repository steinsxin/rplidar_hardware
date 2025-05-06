#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "rplidar.h"

namespace py = pybind11;

PYBIND11_MODULE(RPLidar, m) {
    // 设置模块的文档字符串
    m.doc() = "Python bindings for RPLidar";

    // 绑定 rplidar 类
    py::class_<rplidar>(m, "RPLidar")
        .def(py::init<std::string>(), "Initialize the RPLidar with a serial port and baudrate")
        .def("set_scan_mode", &rplidar::set_scan_mode, "Set the scan mode of the RPLidar")
        .def("checkRPLIDARHealth", &rplidar::checkRPLIDARHealth, "Check the health status of the RPLidar")
        .def("getRPLIDARDeviceInfo", &rplidar::getRPLIDARDeviceInfo, "Get the device information of the RPLidar")
        .def("stop", &rplidar::stop, "Stop the RPLidar")
        .def("start", &rplidar::start, "Start the RPLidar")
        .def("get_data", &rplidar::get_data, "Get scan data from the RPLidar")
        .def("get_angle", &rplidar::get_angle, "Get angle and distance")
        .def("get_max_distance", &rplidar::get_max_distance, "Get maximum distance from last scan")
        .def("get_nodes_count", &rplidar::get_nodes_count, "Get number of measurement nodes in last scan")
        .def("get_scan_duration", &rplidar::get_scan_duration, "Get the duration of a scan in seconds");
}