cmake_minimum_required(VERSION 3.10)
project(RobotArm)

set(CMAKE_CXX_STANDARD 17)



set(TDS_DIR "../tiny-differentiable-simulator")

include_directories(${TDS_DIR}/src)
include_directories(${TDS_DIR}/third_party)
include_directories(${TDS_DIR}/third_party/eigen3)
include_directories(${TDS_DIR}/third_party/tinyxml2/include)

link_directories(${TDS_DIR}/build)


add_executable(my_sim main.cpp) 

# ======== [5] Link to the required TDS libraries ========
# These should exist in your TDS build output (look for .lib or .a files)
target_link_libraries(my_sim
    python/pytinydiffsim_dual
    python/pytinydiffsim
    python/pytinyopengl3
    tinyxml2
    zeromq
    pybullet
    
)
