include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(
    ARMS_SRCS
    JakaServer.cpp
    timespec.cpp
)

add_library(
    arms
    STATIC
    ${ARMS_SRCS}
)

target_include_directories(
    arms
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    arms
    PUBLIC
    proto
    ${CMAKE_SOURCE_DIR}/lib/libjakaAPI.so
)
