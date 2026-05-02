#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "astrall/device/camera.hpp"

namespace astrall {

struct RealCameraConfig {
    std::string udp_url = "udp://0.0.0.0:6000";
    int width = 640;
    int height = 480;
    int channels = 3;
};

class RealCamera final : public Camera {
public:
    explicit RealCamera(const RealCameraConfig& config = {});
    ~RealCamera() override;

    RealCamera(const RealCamera&) = delete;
    RealCamera& operator=(const RealCamera&) = delete;

    ImageFrame getFrame() override;

    bool open();
    void close();
    bool isOpen() const;

private:
    void captureLoop();

    RealCameraConfig config_;
    std::mutex frame_mutex_;
    ImageFrame latest_frame_;
    bool has_frame_ = false;
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace astrall
