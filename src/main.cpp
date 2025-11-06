#include "CameraServer.hpp"
#include "JakaServer.hpp"
#include "Logger.hpp"
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char **argv) {
    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // You can specify a different address and port if needed
        std::string server_address = "127.0.0.1:50551";
        if (argc > 1) {
            server_address = argv[1];
        }

        // Create JakaServer
        jaka_robot::JakaServer jaka_server;

        // Create CameraServer
        camera::CameraServer camera_server;

        // Build and start the gRPC server
        grpc::ServerBuilder builder;

        // Listen on the given address without any authentication mechanism
        builder.AddListeningPort(server_address,
                                 grpc::InsecureServerCredentials());

        // Register services through which we'll communicate with clients
        builder.RegisterService(&jaka_server);
        builder.RegisterService(&camera_server);

        // Finally assemble the server
        std::unique_ptr<grpc::Server> server(builder.BuildAndStart());

        // Wait for the server to shutdown
        LOG_INFO("Server started at %s", server_address.c_str());
        server->Wait();
    } catch (const std::exception &e) {
        std::cerr << "Server failed: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
