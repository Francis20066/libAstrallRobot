#include "astrall/device/real_camera.hpp"

#include <cstring>
#include <stdexcept>

#if ASTRALL_ENABLE_SDK
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace astrall {

RealCamera::RealCamera(const RealCameraConfig& config)
    : config_(config) {}

RealCamera::~RealCamera() {
    close();
}

bool RealCamera::open() {
#if ASTRALL_ENABLE_SDK
    if (running_.load()) {
        return true;
    }

    running_.store(true);
    capture_thread_ = std::thread(&RealCamera::captureLoop, this);
    return true;
#else
    throw std::runtime_error("RealCamera requires ASTRALL_ENABLE_SDK=ON");
#endif
}

void RealCamera::close() {
#if ASTRALL_ENABLE_SDK
    if (!running_.load()) {
        return;
    }

    running_.store(false);
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
#endif
}

bool RealCamera::isOpen() const {
    return running_.load();
}

ImageFrame RealCamera::getFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!has_frame_) {
        ImageFrame frame;
        frame.width = config_.width;
        frame.height = config_.height;
        frame.channels = config_.channels;
        frame.data.resize(
            static_cast<std::size_t>(config_.width) * config_.height * config_.channels, 0);
        return frame;
    }
    return latest_frame_;
}

void RealCamera::captureLoop() {
#if ASTRALL_ENABLE_SDK
    cv::VideoCapture cap(config_.udp_url, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        running_.store(false);
        return;
    }

    cv::Mat frame;
    while (running_.load()) {
        if (!cap.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        cv::Mat bgr;
        if (frame.channels() == 1) {
            cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
        } else if (frame.channels() == 4) {
            cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
        } else {
            bgr = frame;
        }

        if (bgr.cols != config_.width || bgr.rows != config_.height) {
            cv::resize(bgr, bgr, cv::Size(config_.width, config_.height));
        }

        ImageFrame image_frame;
        image_frame.width = bgr.cols;
        image_frame.height = bgr.rows;
        image_frame.channels = bgr.channels();
        const auto data_size = static_cast<std::size_t>(bgr.total()) * bgr.elemSize();
        image_frame.data.resize(data_size);
        std::memcpy(image_frame.data.data(), bgr.data, data_size);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = std::move(image_frame);
            has_frame_ = true;
        }
    }

    cap.release();
#endif
}

}  // namespace astrall
