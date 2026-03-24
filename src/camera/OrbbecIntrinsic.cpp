#include "config.hpp"
#include <libobsensor/ObSensor.hpp>

void printIntrinsics(const OBCameraIntrinsic &intrinsic) {
    std::cout << "  Width: " << intrinsic.width << std::endl;
    std::cout << "  Height: " << intrinsic.height << std::endl;
    std::cout << "  FX: " << intrinsic.fx << std::endl;
    std::cout << "  FY: " << intrinsic.fy << std::endl;
    std::cout << "  CX: " << intrinsic.cx << std::endl;
    std::cout << "  CY: " << intrinsic.cy << std::endl;
}

int main(int argc, char *argv[]) {
    // Check arguments
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <device_serial_number>"
                  << std::endl;
        return -1;
    }

    // Query device and find by serial number
    ob::Context ctx;
    auto devList = ctx.queryDeviceList();
    std::shared_ptr<ob::Device> device;
    try {
        device = devList->getDeviceBySN(argv[1]);
    } catch (const ob::Error &e) {
        std::cerr << "SDK Exception: " << e.what() << std::endl;
        std::cerr << "Available devices:" << std::endl;
        for (uint32_t i = 0; i < devList->getCount(); i++) {
            auto dev = devList->getDevice(i);
            std::cerr << "  " << dev->getDeviceInfo()->serialNumber()
                      << std::endl;
        }
        return -1;
    }

    // Initialize pipeline
    auto pipeline = std::make_unique<ob::Pipeline>(device);

    // Get color stream
    auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    auto color_profile = color_profiles->getVideoStreamProfile(
        camera::IMAGE_WIDTH, camera::IMAGE_HEIGHT, OB_FORMAT_RGB888,
        camera::STREAM_FPS);
    auto color_intrinsic = color_profile->getIntrinsic();
    std::cout << "Color Intrinsic:" << std::endl;
    printIntrinsics(color_intrinsic);

    // Get depth stream
    auto depth_profiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
    auto depth_profile = depth_profiles->getVideoStreamProfile(
        camera::IMAGE_WIDTH, camera::IMAGE_HEIGHT, OB_FORMAT_Y16,
        camera::STREAM_FPS);
    auto depth_intrinsic = depth_profile->getIntrinsic();
    std::cout << "Depth Intrinsic:" << std::endl;
    printIntrinsics(depth_intrinsic);

    return 0;
}
