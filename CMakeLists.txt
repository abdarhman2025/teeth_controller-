cmake_minimum_required(VERSION 3.8)
project(teeth_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(ament_cmake REQUIRED)


cmake_minimum_required(VERSION 3.8)
project(teeth_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(ament_cmake REQUIRED)

# ================================================
# Include xArm C++ SDK headers
# ================================================
# IMPORTANT: Adjust the relative path if needed
include_directories(
  ${CMAKE_SOURCE_DIR}/../../xArm-CPLUS-SDK/include
)

# ================================================
# Build your executable
# ================================================
add_executable(teeth_exec src/main.cpp)

# Link pthread (required by SDK)
target_link_libraries(teeth_exec pthread)

# ================================================
# Install executable so 'ros2 run' works
# ================================================
install(TARGETS teeth_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


