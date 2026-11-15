cmake_minimum_required(VERSION 3.16)

project(RTK LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_executable(rtk
    src--linux/AngleChange.cpp
    src--linux/CoordinateSystem.cpp
    src--linux/Decode.cpp
    src--linux/EKFliter.cpp
    src--linux/ErrorCorrect.cpp
    src--linux/lambda.cpp
    src--linux/main.cpp
    src--linux/MatrixModule.cpp
    src--linux/ReadCtrlFile.cpp
    src--linux/RTK.cpp
    src--linux/SatPVTCalculation.cpp
    src--linux/sockets.cpp
    src--linux/SPP.cpp
    src--linux/TimeChange.cpp
)

target_include_directories(rtk PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src--linux"
)

target_link_libraries(rtk PRIVATE
    Eigen3::Eigen
    Threads::Threads
)

set_target_properties(rtk PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/linux"
)
