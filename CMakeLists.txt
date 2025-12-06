cmake_minimum_required(VERSION 3.8)
project(teeth_controller)

set(CMAKE_CXX_STANDARD 17)

# ==========================================
# Include xArm C++ SDK headers
# ==========================================
include_directories(
  ${CMAKE_SOURCE_DIR}/thirdparty/xArm-CPLUS-SDK/include
)

# ==========================================
# Collect all SDK .cc files EXCEPT /bak/ folder
# ==========================================
file(GLOB_RECURSE ALL_SDK_SOURCES
    ${CMAKE_SOURCE_DIR}/thirdparty/xArm-CPLUS-SDK/src/*.cc
)

# Remove any .cc files inside "bak" directories
set(XARM_SDK_SOURCES "")
foreach(src ${ALL_SDK_SOURCES})
    if(NOT src MATCHES "/bak/")
        list(APPEND XARM_SDK_SOURCES ${src})
    endif()
endforeach()

# ==========================================
# Build the xArm SDK library
# ==========================================
add_library(xarm_cpp_sdk STATIC
    ${XARM_SDK_SOURCES}
)

target_link_libraries(xarm_cpp_sdk pthread)

# ==========================================
# Build  executable for outer cleaner
# ==========================================
add_executable(outer_cleaner src/outer_main.cpp)

target_link_libraries(outer_cleaner
    xarm_cpp_sdk
    pthread
)

# Build  executable for inner cleaner
# ==========================================
add_executable(inner_cleaner src/inner_main.cpp)

target_link_libraries(inner_cleaner
    xarm_cpp_sdk
    pthread
)
