#include "Logger.hpp"
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#if ARMS_TYPE == 0
#include "JakaServer.hpp"
#elif ARMS_TYPE == 1
#include "TjServer.hpp"
#endif

#if CAMERA_TYPE == 0
#include "CameraServer.hpp"
#endif

// The command line options structure
struct CommandLineOptions {
#if ARMS_TYPE == 0
    std::string arms_address = "192.168.2.200";
#elif ARMS_TYPE == 1
    std::string arms_address = "192.168.1.190";
#else
    std::string arms_address = "127.0.0.1";
#endif
    std::string listening_address = "127.0.0.1:50551";
};

CommandLineOptions parseArgs(int argc, char **argv) {
    // Create default options
    CommandLineOptions opts;

    // Iterate through arguments
    for (int i = 1; i < argc; ++i) {
        // The current argument
        std::string arg = argv[i];

        auto get_next_value = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Error: Missing value for " << name << "\n";
                std::exit(1);
            }
            return argv[++i];
        };

        // Parse known options
        if (arg == "--listening_address" || arg == "-l") {
            opts.listening_address = get_next_value(arg);
        } else if (arg == "--arms_address" || arg == "-a") {
            opts.arms_address = get_next_value(arg);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--listening_address addr:port]"
                      << " [--arms_address ip]\n";
            std::exit(0);
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            std::cerr << "Use --help to see available options.\n";
            std::exit(1);
        }
    }

    return opts;
}

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char **argv) {
    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse command line arguments
    CommandLineOptions options = parseArgs(argc, argv);

    // If no services are enabled, exit
#if ARMS_TYPE < 0 && CAMERA_TYPE < 0
    std::cout << "No services enabled. Exiting.\n";
    return 0;
#else
    try {
        // Build and start the gRPC server
        grpc::ServerBuilder builder;

        // Listen on the given address without any authentication mechanism
        builder.AddListeningPort(options.listening_address,
                                 grpc::InsecureServerCredentials());

#if ARMS_TYPE >= 0
        // Create JakaServer if enabled
        std::cout << "Connecting to arms at " << options.arms_address
                  << "...\n";
#endif
#if ARMS_TYPE == 0
        // Create static JakaServer instance
        static arms::JakaArmServer arms_server(options.arms_address);
#elif ARMS_TYPE == 1
        // Create static TjServer instance
        static arms::TjArmServer arms_server(options.arms_address);
#endif
#if ARMS_TYPE >= 0
        builder.RegisterService(&arms_server);
#else
        std::cout << "Robot arms disabled.\n";
#endif

#if CAMERA_TYPE >= 0
        // Create CameraServer if enabled
        std::cout << "Starting camera server...\n";
#endif
#if CAMERA_TYPE == 0
        // Create static CameraServer instance
        static camera::CameraServer camera_server;
#endif
#if CAMERA_TYPE >= 0
        builder.RegisterService(&camera_server);
#else
        std::cout << "Camera server disabled.\n";
#endif
        // Finally assemble the server
        std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
        // Wait for the server to shutdown
        LOG_INFO("Server started at %s", options.listening_address.c_str());
        server->Wait();

    } catch (const std::exception &e) {
        std::cerr << "Server failed: " << e.what() << std::endl;
        return 1;
    }

    return 0;
#endif
}
