#include "Logger.hpp"
#include "OrbbecDevice.hpp"
#include "config.hpp"
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>

namespace camera {

OrbbecDevice::OrbbecDevice(std::shared_ptr<ob::Device> device)
    : device_(device), createTime_(std::chrono::system_clock::now()) {
    // Initialize serial number
    serialNumber_ = device_->getDeviceInfo()->serialNumber();

    // Initialize pipeline
    pipeline_ = std::make_unique<ob::Pipeline>(device_);
    auto config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT,
                              STREAM_FPS, OB_FORMAT_RGB888);
    pipeline_->start(config);
    LOG_INFO("[Device %s] Pipeline started, camera work commencing.",
             serialNumber_.c_str());
}

OrbbecDevice::~OrbbecDevice() {
    pipeline_->stop();
    LOG_INFO("[Device %s] Pipeline stopped, device destructor called.",
             serialNumber_.c_str());
}

const std::string &OrbbecDevice::getSerialNumber() const {
    return serialNumber_;
}

std::chrono::system_clock::time_point OrbbecDevice::getCreateTime() const {
    return createTime_;
}

std::string OrbbecDevice::getCreateTimeString() const {
    std::time_t t = std::chrono::system_clock::to_time_t(createTime_);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%F %T");
    return ss.str();
}

std::optional<std::unique_ptr<Image>> OrbbecDevice::getOneFrame() {
    try {
        // Get a frame set
        auto frameSet = pipeline_->waitForFrames(1000);
        if (!frameSet) {
            LOG_WARN("[Device %s] No frames received from waitForFrames.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Get color frame
        auto colorFrame =
            frameSet->getFrame(OB_FRAME_COLOR)->as<ob::ColorFrame>();
        if (!colorFrame) {
            LOG_WARN("[Device %s] No color frame available in frame set.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Check frame data
        if (colorFrame->getData() == nullptr ||
            colorFrame->getWidth() != IMAGE_WIDTH ||
            colorFrame->getHeight() != IMAGE_HEIGHT ||
            colorFrame->getDataSize() != IMAGE_SIZE) {
            LOG_WARN("[Device %s] Invalid color frame data.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Copy frame data into Image structure
        auto img = std::make_unique<Image>();
        memcpy(&img->data, colorFrame->getData(), IMAGE_SIZE);
        return std::make_optional(std::move(img));

    } catch (const ob::Error &e) {
        LOG_ERROR("[Device %s] SDK Exception: %s", serialNumber_.c_str(),
                  e.what());
    } catch (const std::exception &e) {
        LOG_ERROR("[Device %s] Standard Exception: %s", serialNumber_.c_str(),
                  e.what());
    }

    return std::nullopt;
}

} // namespace camera
