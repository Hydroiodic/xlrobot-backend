include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(
    ARMS_SRCS
    ErrorParser.cpp
    timespec.cpp
)

if(ARMS_TYPE STREQUAL "jaka")
    list(APPEND ARMS_SRCS JakaServer.cpp)
elseif(ARMS_TYPE STREQUAL "tj")
    list(APPEND ARMS_SRCS TjServer.cpp)
else()
    message(FATAL_ERROR "Unknown ARMS_TYPE='${ARMS_TYPE}', expected 'jaka' or 'tj'")
endif()

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
)

if(ARMS_TYPE STREQUAL "jaka")
    target_link_libraries(
        arms
        PUBLIC
        ${CMAKE_SOURCE_DIR}/lib/libjakaAPI.so
    )
endif()
