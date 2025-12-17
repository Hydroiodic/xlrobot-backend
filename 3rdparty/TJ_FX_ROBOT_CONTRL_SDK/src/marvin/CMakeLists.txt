add_library(
    MarvinSDK
    STATIC
    MarvinSDK.cpp
    Robot.cpp
    ACB.cpp
    FXDG.cpp
    PointSet.cpp
    FileOP.cpp
    FilePortal.cpp
    Parser.cpp
    TCPAgent.cpp
    TCPFileClient.cpp
)

target_link_libraries(
    MarvinSDK
    PUBLIC
    pthread
    rt
)

target_include_directories(
    Kine
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
