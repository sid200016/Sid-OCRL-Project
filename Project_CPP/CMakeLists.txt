cmake_minimum_required(VERSION 3.10)
project(RobotArm)

set(CMAKE_CXX_STANDARD 17)

# Path to the TDS directory (assumed to be sibling to this project)
set(TDS_DIR "../tiny-differentiable-simulator")

# Include directories
include_directories(${TDS_DIR}/src)
include_directories(${TDS_DIR}/examples)
include_directories(${TDS_DIR}/src/math)
include_directories(${TDS_DIR}/src/dynamics)
include_directories(${TDS_DIR}/src/urdf)
include_directories(${TDS_DIR}/src/utils)
include_directories(${TDS_DIR}/src/utils/cuda)
include_directories(${TDS_DIR}/src/utils/sdf_to_mesh)
include_directories(${TDS_DIR}/src/visualizer/meshcat)
include_directories(${TDS_DIR}/src/visualizer/opengl)
include_directories(${TDS_DIR}/src/visualizer/pybullet)
include_directories(${TDS_DIR}/src/visualizer/tinyrenderer)
include_directories(${TDS_DIR}/third_party)
include_directories(${TDS_DIR}/third_party/tinyobjloader)
include_directories(${TDS_DIR}/third_party/eigen3)
include_directories(${TDS_DIR}/third_party/tinyxml2/include)
include_directories(${TDS_DIR}/third_party/cpp_base64/include/cpp_base64)
include_directories(${TDS_DIR}/third_party/zeromq/include)
include_directories(${TDS_DIR}/third_party/zeromq/include_linux)
include_directories(${TDS_DIR}/third_party/dear_imgui)
include_directories(${TDS_DIR}/third_party/dear_imgui/opengl3)
include_directories(${TDS_DIR}/third_party/gtest/include)
include_directories(${TDS_DIR}/third_party/crossguid/crossguid)


# Link directories for all .so/.a
link_directories(
    ${TDS_DIR}/build/rbdl
    ${TDS_DIR}/build/python
    ${TDS_DIR}/build/cpp_base64
    ${TDS_DIR}/build/zeromq
    ${TDS_DIR}/build/dear_imgui
    ${TDS_DIR}/build/gtest
    ${TDS_DIR}/build/crossguid
    ${TDS_DIR}/build/tinyxml2
    ${TDS_DIR}/build/opengl
)

# Your executable
add_executable(my_sim main.cpp)

# Link against all the relevant built libs
target_link_libraries(my_sim
    rbdl-static
    rbdl-urdfreader
    pytinydiffsim_dual
    pytinydiffsim
    pytinyopengl3
    cpp_base64
    zmq
    dear_imgui
    gtest
    gtestmain
    crossguid
    tinyxml2
    opengl_window
    dl
)

