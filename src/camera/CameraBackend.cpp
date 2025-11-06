#include "CameraBackend.hpp"
#include "Logger.hpp"
#include <memory>
#include <optional>

namespace camera {

// Initialize static members
ob::Context CameraBackend::ctx;
std::mutex CameraBackend::mtx;
std::map<std::string, std::shared_ptr<OrbbecDevice>> CameraBackend::devs;

CameraBackend::CameraBackend() {
    // Get the list of connected devices
    auto devList = ctx.queryDeviceList();
    LOG_INFO("Got %d devices for CameraBackend.", devList->getCount());

    // Lock the mutex to protect access to the pipes map
    std::lock_guard<std::mutex> lock(mtx);

    // Check if pipes are already initialized
    if (devs.size() > 0) {
        LOG_INFO("Pipes already initialized.");
        return;
    }

    // Get the number of connected devices
    int devCount = devList->getCount();
    for (int i = 0; i < devCount; i++) {
        // Get the device from device list
        auto dev = devList->getDevice(i);
        // Add the pipeline to the map of pipelines
        devs.insert({std::string(dev->getDeviceInfo()->getSerialNumber()),
                     std::make_shared<OrbbecDevice>(dev)});
    }
}

CameraBackend::~CameraBackend() {}

const std::vector<std::string>
CameraBackend::getConnectedDeviceSerialNumbers() const {
    std::vector<std::string> result;
    for (const auto &pair : devs) {
        result.push_back(pair.first);
    }
    return result;
}

std::optional<std::unique_ptr<Image>>
CameraBackend::getImage(const std::string &serialNumber) {
    // Try to find the device in the map
    auto it = devs.find(serialNumber);
    if (it == devs.end()) {
        LOG_ERROR("Device with serial number %s not found.",
                  serialNumber.c_str());
        return std::nullopt;
    }

    // Try to get one frame from the device
    auto device = it->second;
    auto frame = device->getOneFrame();
    if (!frame) {
        LOG_ERROR("Failed to get frame from device %s.", serialNumber.c_str());
        return std::nullopt;
    }

    return std::move(frame);
}

} // namespace camera
