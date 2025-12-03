#pragma once

// This file defines mathematical constants and utility functions
// for angle conversions.

namespace angle {

// Mathematical constants
constexpr double PI = 3.14159265358979323846;

// Convert degrees to radians
inline double degreesToRadians(double degrees) {
    return degrees * (PI / 180.0);
}

// Convert radians to degrees
inline double radiansToDegrees(double radians) {
    return radians * (180.0 / PI);
}

} // namespace angle
