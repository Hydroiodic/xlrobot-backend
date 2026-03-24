#include "Logger.hpp"
#include "OrbbecDevice.hpp"
#include "config.hpp"
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>

namespace camera {

static const std::string GEMINI_336L_NAME = "Orbbec Gemini 336L";

OrbbecDevice::OrbbecDevice(std::shared_ptr<ob::Device> device)
    : device_(device), config_(std::make_shared<ob::Config>()),
      createTime_(std::chrono::system_clock::now()) {
    // Initialize serial number and judge whether depth stream is enabled
    serialNumber_ = device_->getDeviceInfo()->serialNumber();
    depth_enabled_ = device_->getDeviceInfo()->name() == GEMINI_336L_NAME;

    // Initialize pipeline and config
    pipeline_ = std::make_unique<ob::Pipeline>(device_);

    // Enable color stream
    auto color_profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
    auto color_profile = color_profiles->getVideoStreamProfile(
        IMAGE_WIDTH, IMAGE_HEIGHT, OB_FORMAT_RGB888, STREAM_FPS);
    config_->enableStream(color_profile);

    // Depth stream is only enabled for Gemini 336L
    if (depth_enabled_) {
        // Enable depth stream
        auto depth_profiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
        auto depth_profile = depth_profiles->getVideoStreamProfile(
            IMAGE_WIDTH, IMAGE_HEIGHT, OB_FORMAT_Y16, STREAM_FPS);
        config_->enableStream(depth_profile);
        // Enable accelerometer stream
        if (device->isPropertySupported(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL,
                                        OB_PERMISSION_READ)) {
            config_->setAlignMode(ALIGN_D2C_HW_MODE);
        } else {
            config_->setAlignMode(ALIGN_D2C_SW_MODE);
        }
    }

    // // Setup device changed callback
    // ctx_->setDeviceChangedCallback(
    //     [this](std::shared_ptr<ob::DeviceList> removed_devices,
    //            std::shared_ptr<ob::DeviceList> added_devices) {
    //         if (removed_devices->getDeviceBySN(this->serialNumber_.c_str()))
    //         {
    //             this->pipeline_->stop();
    //         } else if (added_devices->getDeviceBySN(
    //                        this->serialNumber_.c_str())) {
    //             this->pipeline_ = std::make_unique<ob::Pipeline>();
    //             this->pipeline_->start(this->config_);
    //         }
    //     });

    this->pipeline_->start(this->config_);
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

        // Depth stream is only enabled for Gemini 336L
        auto depthImg = std::make_unique<Image>();
        if (depth_enabled_) {
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
            memcpy(&depthImg->data, depthFrame->getData(),
                   IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(uint16_t));
        }

        // Return the result
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
