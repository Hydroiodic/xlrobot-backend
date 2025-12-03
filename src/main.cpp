#include "CameraServer.hpp"
#include "Logger.hpp"
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

// 0: jaka, 1: tj
#ifndef ARMS_TYPE
#define ARMS_TYPE 0
#endif

#if ARMS_TYPE == 0
#include "JakaServer.hpp"
#elif ARMS_TYPE == 1
#include "TjServer.hpp"
#else
static_assert(false, "Unknown ARMS_TYPE");
#endif

// The command line options structure
struct CommandLineOptions {
    bool enable_arms = true;
    bool enable_camera = true;
    std::string arms_address = "192.168.2.200";
    std::string listening_address = "127.0.0.1:50551";
};

CommandLineOptions parseArgs(int argc, char **argv) {
    // Create default options
    CommandLineOptions opts;

    // To parse boolean values from strings
    auto parse_bool = [](const std::string &value) -> bool {
        // Convert to lower case for comparison
        auto value_lower = value;
        std::transform(value_lower.begin(), value_lower.end(),
                       value_lower.begin(), ::tolower);

        // Check common true/false representations
        if (value_lower == "1" || value_lower == "true" ||
            value_lower == "on" || value_lower == "yes")
            return true;
        if (value_lower == "0" || value_lower == "false" ||
            value_lower == "off" || value_lower == "no")
            return false;
        std::cerr << "Invalid boolean value: " << value
                  << " (use true/false, on/off, 1/0)\n";
        std::exit(1);
    };

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
        } else if (arg == "--enable_arms") {
            opts.enable_arms = parse_bool(get_next_value(arg));
        } else if (arg == "--enable_camera") {
            opts.enable_camera = parse_bool(get_next_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--listening_address addr:port]"
                      << " [--arms_address ip]" << " [--enable_arms true|false]"
                      << " [--enable_camera true|false]\n";
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
    if (!options.enable_arms && !options.enable_camera) {
        std::cout << "No services enabled. Exiting.\n";
        return 0;
    }

    try {
        // Build and start the gRPC server
        grpc::ServerBuilder builder;

        // Listen on the given address without any authentication mechanism
        builder.AddListeningPort(options.listening_address,
                                 grpc::InsecureServerCredentials());

        // Create JakaServer if enabled
        if (options.enable_arms) {
            std::cout << "Connecting to arms at " << options.arms_address
                      << "...\n";
#if ARMS_TYPE == 0
            // Create static JakaServer instance
            static arms::JakaArmServer arms_server(options.arms_address);
#elif ARMS_TYPE == 1
            // Create static TjServer instance
            static arms::TjArmServer arms_server(options.arms_address);
#endif
            builder.RegisterService(&arms_server);
        } else {
            std::cout << "Jaka arms disabled.\n";
        }

        // Create CameraServer if enabled
        if (options.enable_camera) {
            std::cout << "Starting camera server...\n";

            // Create static CameraServer instance
            static camera::CameraServer camera_server;
            builder.RegisterService(&camera_server);
        } else {
            std::cout << "Camera server disabled.\n";
        }

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
}
