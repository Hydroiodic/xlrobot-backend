#include "CameraServer.hpp"
#include <grpcpp/grpcpp.h>

namespace camera {

CameraServer::CameraServer() {}

CameraServer::~CameraServer() {}

grpc::Status CameraServer::GetConnectedDeviceSerialNumbers(
    grpc::ServerContext *context, const camera::EmptyRequest *request,
    camera::DeviceListResponse *response) {

    try {
        auto serialNumbers = cameraBackend.getConnectedDeviceSerialNumbers();

        for (const auto &serial : serialNumbers) {
            response->add_serial_numbers(serial);
        }

        return grpc::Status::OK;
    } catch (const std::exception &e) {
        return grpc::Status(grpc::StatusCode::INTERNAL,
                            "Failed to get connected devices: " +
                                std::string(e.what()));
    }
}

grpc::Status CameraServer::GetImage(grpc::ServerContext *context,
                                    const camera::GetImageRequest *request,
                                    camera::ImageResponse *response) {
    try {
        std::string serialNumber = request->serial_number();

        if (serialNumber.empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "Serial number cannot be empty");
        }

        auto imageOpt = cameraBackend.getImage(serialNumber);

        if (imageOpt.has_value()) {
            auto &image = imageOpt.value();

            // Set image data
            response->set_success(true);
            response->set_color_data(image.first->data, IMAGE_SIZE);
            response->set_depth_data(image.second->data, IMAGE_WIDTH *
                                                             IMAGE_HEIGHT *
                                                             sizeof(uint16_t));
            response->set_width(IMAGE_WIDTH);
            response->set_height(IMAGE_HEIGHT);
        } else {
            response->set_success(false);
            response->set_error_message("No image available for device: " +
                                        serialNumber);
        }

        return grpc::Status::OK;
    } catch (const std::exception &e) {
        response->set_success(false);
        response->set_error_message(e.what());
        return grpc::Status(grpc::StatusCode::INTERNAL,
                            "Failed to get image: " + std::string(e.what()));
    }
}

} // namespace camera
