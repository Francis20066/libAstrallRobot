#include <cstring>
#include <memory>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "astrall/device/camera.hpp"
#include "astrall/device/dummy_camera.hpp"
#include "astrall/device/dummy_radar.hpp"
#include "astrall/device/radar.hpp"
#include "astrall/device/real_camera.hpp"

namespace py = pybind11;

namespace {

py::array_t<std::uint8_t> frameToNumpy(const astrall::ImageFrame& frame) {
    py::array_t<std::uint8_t> array({frame.height, frame.width, frame.channels});
    std::memcpy(array.mutable_data(), frame.data.data(), frame.data.size() * sizeof(std::uint8_t));
    return array;
}

py::array_t<float> cloudToNumpy(const astrall::PointCloud& cloud) {
    py::array_t<float> array({static_cast<py::ssize_t>(cloud.points.size()), py::ssize_t{4}});
    auto* out = static_cast<float*>(array.mutable_data());
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        out[i * 4 + 0] = cloud.points[i].x;
        out[i * 4 + 1] = cloud.points[i].y;
        out[i * 4 + 2] = cloud.points[i].z;
        out[i * 4 + 3] = cloud.points[i].intensity;
    }
    return array;
}

}  // namespace

void bind_device(py::module_& m) {
    py::class_<astrall::ImageFrame>(
        m,
        "ImageFrame",
        "C++ camera frame domain object. Camera.get_frame() returns a copied numpy array for Python convenience.")
        .def(py::init<>())
        .def_readwrite("width", &astrall::ImageFrame::width)
        .def_readwrite("height", &astrall::ImageFrame::height)
        .def_readwrite("channels", &astrall::ImageFrame::channels)
        .def_readwrite("data", &astrall::ImageFrame::data);

    py::class_<astrall::Camera, std::shared_ptr<astrall::Camera>>(m, "Camera")
        .def("get_frame", [](astrall::Camera& camera) {
            return frameToNumpy(camera.getFrame());
        }, "Return a copied numpy uint8 array with shape (height, width, channels).");

    py::class_<astrall::DummyCamera, astrall::Camera, std::shared_ptr<astrall::DummyCamera>>(m, "DummyCamera")
        .def(py::init<int, int, int>(), py::arg("width") = 640, py::arg("height") = 480, py::arg("channels") = 3);

    py::class_<astrall::RealCamera, astrall::Camera, std::shared_ptr<astrall::RealCamera>>(m, "RealCamera")
        .def("open", &astrall::RealCamera::open)
        .def("close", &astrall::RealCamera::close)
        .def("is_open", &astrall::RealCamera::isOpen);

    py::class_<astrall::PointXYZI>(m, "PointXYZI")
        .def(py::init<>())
        .def_readwrite("x", &astrall::PointXYZI::x)
        .def_readwrite("y", &astrall::PointXYZI::y)
        .def_readwrite("z", &astrall::PointXYZI::z)
        .def_readwrite("intensity", &astrall::PointXYZI::intensity);

    py::class_<astrall::PointCloud>(
        m,
        "PointCloud",
        "C++ point-cloud domain object. Radar.get_pointcloud() returns a copied numpy array for Python convenience.")
        .def(py::init<>())
        .def_readwrite("points", &astrall::PointCloud::points);

    py::class_<astrall::Radar, std::shared_ptr<astrall::Radar>>(m, "Radar")
        .def("get_pointcloud", [](astrall::Radar& radar) {
            return cloudToNumpy(radar.getPointCloud());
        }, "Return a copied numpy float32 array with shape (point_count, 4). Production LiDAR must use a ROS2 driver.");

    py::class_<astrall::DummyRadar, astrall::Radar, std::shared_ptr<astrall::DummyRadar>>(m, "DummyRadar")
        .def(py::init<int>(), py::arg("point_count") = 1024);
}
