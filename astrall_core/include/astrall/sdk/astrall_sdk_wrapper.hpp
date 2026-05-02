#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "astrall/common/geometry.hpp"
#include "astrall_sdk.hpp"

namespace astrall {

struct AstrallSdkConfig {
    // The current Astrall C API does not expose an IP setter. Keep these values
    // in config so ROS2 parameters and diagnostics document the expected network.
    std::string sdk_ip = "10.18.0.200";
    std::string robot_ip = "10.18.0.100";
    std::uint32_t init_timeout_ms = 60000;
    std::uint32_t command_timeout_ms = 20;
};

class AstrallSdkWrapper {
public:
    AstrallSdkWrapper();
    ~AstrallSdkWrapper();

    AstrallSdkWrapper(const AstrallSdkWrapper&) = delete;
    AstrallSdkWrapper& operator=(const AstrallSdkWrapper&) = delete;

    bool init(const AstrallSdkConfig& config);
    void shutdown() noexcept;

    bool requestControl();
    bool releaseControl();

    bool move(const Twist2D& cmd);
    // Compatibility convenience overload. Prefer move(const Twist2D&) for new code.
    bool move(double vx, double vy, double w);
    bool stop();

    std::optional<AstrallImuData> latestImu() const;
    std::optional<AstrallSportData> latestSport() const;
    std::optional<AstrallJoystickData> latestJoystick() const;
    std::optional<AstrallSdkStatus> latestSdkStatus() const;
    std::optional<AstrallSystemStatus> systemStatus() const;

    bool subscribeImu(int freq_hz);
    bool subscribeSport(int freq_hz);
    bool subscribeJoystick(int freq_hz);
    bool subscribeCamera(int freq_hz);

    Result lastResult() const;
    bool hasControlAuthority() const;

private:
    static Frequency frequencyFromHz(int freq_hz);
    void rememberResult(Result result);
    bool succeededAndRemember(Result result);

    AstrallSdkConfig config_;
    std::unique_ptr<Client> client_;
    mutable std::mutex api_mutex_;
    mutable std::mutex data_mutex_;
    std::optional<AstrallImuData> latest_imu_;
    std::optional<AstrallSportData> latest_sport_;
    std::optional<AstrallJoystickData> latest_joystick_;
    std::optional<AstrallSdkStatus> latest_sdk_status_;
    Result last_result_ = Result::failed;
};

}  // namespace astrall
