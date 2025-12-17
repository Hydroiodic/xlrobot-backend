add_library(
    Kine
    STATIC
    AxisPln.cpp
    FXDG.cpp
    FXDT.c
    FXFLT.c
    FXMath.c
    FXMatrix.c
    FxRobot.c
    FXSpln.cpp
    LoadIdenPub.cpp
    O3Polynorm.cpp
    PointSet.cpp
)

target_link_libraries(
    Kine
    PUBLIC
    pthread
    rt
)

target_include_directories(
    Kine
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
