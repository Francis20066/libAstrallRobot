#include "astrall/runtime.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "astrall/backend/backend_factory.hpp"
#include "astrall/device/dummy_camera.hpp"
#include "astrall/device/dummy_radar.hpp"
#include "astrall/device/real_camera.hpp"
#include "astrall/planning/straight_line_planner.hpp"

namespace astrall {

namespace {

template <typename T>
T getOr(const YAML::Node& node, const char* key, const T& fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<T>();
}

std::shared_ptr<Planner> createPlanner(const YAML::Node& config) {
    const std::string planner_type = getOr(config, "type", std::string("straight_line"));
    if (planner_type != "straight_line") {
        throw std::runtime_error("Unsupported planner type: " + planner_type);
    }
    return std::make_shared<StraightLinePlanner>(getOr(config, "waypoint_count", 20));
}

std::shared_ptr<Camera> createCamera(const YAML::Node& config) {
    const std::string camera_type = getOr(config, "type", std::string("dummy"));
    if (camera_type == "dummy") {
        return std::make_shared<DummyCamera>(
            getOr(config, "width", 640),
            getOr(config, "height", 480),
            getOr(config, "channels", 3));
    }
    if (camera_type == "real") {
        RealCameraConfig cfg;
        cfg.udp_url = getOr(config, "udp_url", std::string("udp://0.0.0.0:6000"));
        cfg.width = getOr(config, "width", 640);
        cfg.height = getOr(config, "height", 480);
        cfg.channels = getOr(config, "channels", 3);
        auto camera = std::make_shared<RealCamera>(cfg);
        camera->open();
        return camera;
    }
    throw std::runtime_error("Unsupported camera type: " + camera_type);
}

std::shared_ptr<Radar> createRadar(const YAML::Node& config) {
    const std::string radar_type = getOr(config, "type", std::string("dummy"));
    if (radar_type != "dummy") {
        throw std::runtime_error("Unsupported radar type: " + radar_type);
    }
    return std::make_shared<DummyRadar>(getOr(config, "point_count", 1024));
}

}  // namespace

std::shared_ptr<Runtime> Runtime::fromConfig(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    auto runtime = std::shared_ptr<Runtime>(new Runtime());

    BackendFactoryConfig backend_config;
    backend_config.kind = backendKindFromString(getOr(config["robot"], "backend", std::string("sim")));
    backend_config.sim_dt = getOr(config["backend"]["sim"], "dt", backend_config.sim_dt);

    const YAML::Node real_config = config["backend"]["real"];
    backend_config.real.sdk_ip = getOr(real_config, "sdk_ip", backend_config.real.sdk_ip);
    backend_config.real.robot_ip = getOr(real_config, "robot_ip", backend_config.real.robot_ip);
    backend_config.real.init_timeout_ms = getOr(real_config, "init_timeout_ms", backend_config.real.init_timeout_ms);
    backend_config.real.command_timeout_ms = getOr(real_config, "command_timeout_ms", backend_config.real.command_timeout_ms);
    backend_config.real.imu_frequency_hz = getOr(real_config, "imu_frequency_hz", backend_config.real.imu_frequency_hz);
    backend_config.real.sport_frequency_hz = getOr(real_config, "sport_frequency_hz", backend_config.real.sport_frequency_hz);
    backend_config.real.sdk_quaternion_order = getOr(real_config, "sdk_quaternion_order", backend_config.real.sdk_quaternion_order);
    backend_config.real.request_control = getOr(real_config, "request_control", backend_config.real.request_control);
    runtime->backend_ = createBackend(backend_config);

    const YAML::Node controller_config = config["controller"];
    runtime->controller_ = std::make_shared<Controller>(
        runtime->backend_,
        getOr(controller_config, "max_v", 1.0),
        getOr(controller_config, "max_w", 1.0),
        getOr(controller_config, "position_tolerance", 0.1),
        getOr(controller_config, "angle_tolerance", 0.1));

    runtime->planner_ = createPlanner(config["planner"]);
    runtime->camera_ = createCamera(config["camera"]);
    runtime->radar_ = createRadar(config["radar"]);

    runtime->navigator_ = std::make_shared<Navigator>(
        runtime->planner_, runtime->controller_, runtime->backend_);
    runtime->state_machine_ = std::make_shared<StateMachine>(runtime->navigator_);

    return runtime;
}

std::shared_ptr<Backend> Runtime::backend() {
    return backend_;
}

std::shared_ptr<Controller> Runtime::controller() {
    return controller_;
}

std::shared_ptr<Planner> Runtime::planner() {
    return planner_;
}

std::shared_ptr<Navigator> Runtime::navigator() {
    return navigator_;
}

std::shared_ptr<StateMachine> Runtime::stateMachine() {
    return state_machine_;
}

std::shared_ptr<Camera> Runtime::camera() {
    return camera_;
}

std::shared_ptr<Radar> Runtime::radar() {
    return radar_;
}

}  // namespace astrall
