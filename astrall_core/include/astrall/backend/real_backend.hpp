#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "astrall/backend/backend.hpp"

namespace astrall {

class AstrallSdkWrapper;

struct RealBackendConfig {
    // The Astrall C API currently does not expose IP setters; these document
    // and diagnose the expected network endpoint.
    std::string sdk_ip = "10.18.0.200";
    std::string robot_ip = "10.18.0.100";
    std::uint32_t init_timeout_ms = 60000;
    std::uint32_t command_timeout_ms = 20;
    int imu_frequency_hz = 250;
    int sport_frequency_hz = 250;
    int camera_frequency_hz = 25;
    std::string sdk_quaternion_order = "xyzw";
    bool request_control = true;
};

// SDK-backed chassis backend. It owns the Astrall SDK lifecycle and exposes
// base telemetry/control to core and ROS2 callers. The pose returned here is
// SDK odometry telemetry for demos/minimal runtime paths; production navigation
// should use ROS2 localization odom/TF from FAST-LIO or another estimator.
class RealBackend final : public Backend {
public:
    explicit RealBackend(const RealBackendConfig& config);
    // Compatibility constructor for older callers. New code should pass
    // RealBackendConfig because the backend is SDK-backed, not serial-backed.
    [[deprecated("Use RealBackendConfig; RealBackend is SDK-backed, not serial-backed.")]]
    RealBackend(std::string port = "/dev/ttyUSB0", int baudrate = 115200);
    ~RealBackend() override;

    void sendVelocity(const Twist2D& cmd) override;
    Pose2D getCurrentPose() const override;
    void stop() override;
    std::optional<BackendImuData> latestImu() const override;
    std::optional<WheelSpeeds> latestWheelSpeeds() const override;
    BackendStatus status() const override;
    bool hasControlAuthority() const override;

    const RealBackendConfig& config() const;
    const std::string& port() const;
    int baudrate() const;

private:
    void initializeSdk();
    void applyCachedSystemStatus(BackendStatus& backend_status) const;

    RealBackendConfig config_;
    std::string port_;
    int baudrate_ = 115200;
    Pose2D pose_;
    bool initialized_ = false;
    mutable std::mutex status_mutex_;
    mutable BackendStatus cached_system_status_;
    mutable bool has_cached_system_status_ = false;
    mutable std::chrono::steady_clock::time_point last_system_status_refresh_{};
#if ASTRALL_ENABLE_SDK
    std::unique_ptr<AstrallSdkWrapper> sdk_;
#endif
};

}  // namespace astrall
