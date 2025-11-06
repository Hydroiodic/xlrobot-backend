#pragma once

#include "OrbbecDevice.hpp"
#include <libobsensor/ObSensor.hpp>
#include <mutex>
#include <string>
#include <vector>

namespace camera {

class CameraBackend {
  protected:
    static ob::Context ctx;
    static std::mutex mtx;
    static std::map<std::string, std::shared_ptr<OrbbecDevice>> devs;

  public:
    CameraBackend();
    ~CameraBackend();

    const std::vector<std::string> getConnectedDeviceSerialNumbers() const;

    std::optional<std::unique_ptr<Image>>
    getImage(const std::string &serialNumber);
};

} // namespace camera
