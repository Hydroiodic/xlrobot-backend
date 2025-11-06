#pragma once

#include <iostream>
#include <string>
#include <mutex>
#include <chrono>
#include <ctime>
#include <fstream>
#include <vector>

// Simple thread-safe singleton logger that writes to console and file
class Logger {
public:
    // Delete copy constructor and assignment operator
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    // Initialize the logger with a file path
    void init(const std::string& log_filepath) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
        file_stream_.open(log_filepath, std::ios::out | std::ios::app);
        if (!file_stream_.is_open()) {
            fprintf(stderr, "[FATAL] Failed to open log file: %s\n", log_filepath.c_str());
        }
    }

    template<typename... Args>
    void info(const char* format, Args... args) {
        log("INFO", format, args...);
    }

    template<typename... Args>
    void warn(const char* format, Args... args) {
        log("WARN", format, args...);
    }

    template<typename... Args>
    void error(const char* format, Args... args) {
        log("ERROR", format, args...);
    }

private:
    Logger() = default;
    ~Logger() {
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
    }

    template<typename... Args>
    void log(const char* level, const char* format, Args... args) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Get timestamp
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        char time_buf[20];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&in_time_t));

        char header_buf[128];
        snprintf(header_buf, sizeof(header_buf), "[%s.%03ld] [%s] ", time_buf, ms.count(), level);

        // Format message safely
        int size = std::snprintf(nullptr, 0, format, args...);
        std::vector<char> message_buf(size + 1);
        std::snprintf(message_buf.data(), message_buf.size(), format, args...);

        // Write to stderr
        fprintf(stderr, "%s%s\n", header_buf, message_buf.data());

        // Write to file if open
        if (file_stream_.is_open()) {
            file_stream_ << header_buf << message_buf.data() << "\n";
            file_stream_.flush(); // Ensure it's written immediately
        }
    }

    std::mutex mutex_;
    std::ofstream file_stream_;
};

// Define a macro for easier access
#define LOG_INFO(...)  Logger::getInstance().info(__VA_ARGS__)
#define LOG_WARN(...)  Logger::getInstance().warn(__VA_ARGS__)
#define LOG_ERROR(...) Logger::getInstance().error(__VA_ARGS__)
