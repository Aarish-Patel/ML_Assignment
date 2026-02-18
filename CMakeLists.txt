cmake_minimum_required(VERSION 3.16)

project(ML_Assignment)

set(CMAKE_CXX_STANDARD 17)

# =====================
# ONNX Runtime
# =====================

set(ONNXRUNTIME_DIR ${CMAKE_SOURCE_DIR}/external/onnxruntime)

include_directories(
    include
    ${ONNXRUNTIME_DIR}/include
)

link_directories(
    ${ONNXRUNTIME_DIR}/lib
)

# =====================
# Source files
# =====================

file(GLOB SRC_FILES src/*.cpp)

# =====================
# Main executable
# =====================

add_executable(main_app
    ${SRC_FILES}
    apps/main.cpp
)

target_link_libraries(main_app onnxruntime)
