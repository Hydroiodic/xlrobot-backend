#include "Logger.hpp"
#include "OrbbecDevice.hpp"
#include "config.hpp"
#include "libobsensor/h/ObTypes.h"
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>

namespace camera {

OrbbecDevice::OrbbecDevice(std::shared_ptr<ob::Device> device)
    : device_(device), createTime_(std::chrono::system_clock::now()) {
    // Initialize serial number
    serialNumber_ = device_->getDeviceInfo()->serialNumber();

    // Initialize pipeline
    pipeline_ = std::make_unique<ob::Pipeline>(device_);
    auto config = std::make_shared<ob::Config>();

    // Enable color stream
    auto color_profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
    auto color_profile = color_profiles->getVideoStreamProfile(
        IMAGE_WIDTH, IMAGE_HEIGHT, OB_FORMAT_RGB888, STREAM_FPS);
    config->enableStream(color_profile);

    // Enable depth stream
    auto depth_profiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
    auto depth_profile = depth_profiles->getVideoStreamProfile(
        IMAGE_WIDTH, IMAGE_HEIGHT, OB_FORMAT_Y16, STREAM_FPS);
    config->enableStream(depth_profile);

    // Enable accelerometer stream
    if (device->isPropertySupported(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL,
                                    OB_PERMISSION_READ)) {
        config->setAlignMode(ALIGN_D2C_HW_MODE);
    } else {
        config->setAlignMode(ALIGN_D2C_SW_MODE);
    }

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

std::optional<std::pair<std::unique_ptr<Image>, std::unique_ptr<Image>>>
OrbbecDevice::getOneFrame() {
    try {
        // Get a frame set
        auto frameSet = pipeline_->waitForFrames(1000);
        if (!frameSet) {
            LOG_WARN("[Device %s] No frames received from waitForFrames.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Get color frame
        auto colorFrame = frameSet->colorFrame();
        if (!colorFrame) {
            LOG_WARN("[Device %s] No color frame available in frame set.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Check color frame data
        if (colorFrame->getData() == nullptr ||
            colorFrame->getWidth() != IMAGE_WIDTH ||
            colorFrame->getHeight() != IMAGE_HEIGHT ||
            colorFrame->getDataSize() != IMAGE_SIZE) {
            LOG_WARN("[Device %s] Invalid color frame data.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Copy frame data into Image structure
        auto colorImg = std::make_unique<Image>();
        memcpy(&colorImg->data, colorFrame->getData(), IMAGE_SIZE);

        // Get depth frame
        auto depthFrame = frameSet->depthFrame();
        if (!depthFrame) {
            LOG_WARN("[Device %s] No depth frame available in frame set.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Check depth frame data
        if (depthFrame->getData() == nullptr ||
            depthFrame->getWidth() != IMAGE_WIDTH ||
            depthFrame->getHeight() != IMAGE_HEIGHT ||
            depthFrame->getFormat() != OB_FORMAT_Y16) {
            LOG_WARN("[Device %s] Invalid depth frame data.",
                     serialNumber_.c_str());
            return std::nullopt;
        }

        // Copy frame data into Image structure
        auto depthImg = std::make_unique<Image>();
        memcpy(&depthImg->data, depthFrame->getData(),
               IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(uint16_t));

        return std::make_optional(
            std::make_pair(std::move(colorImg), std::move(depthImg)));

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
