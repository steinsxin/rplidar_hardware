cmake_minimum_required(VERSION 3.10)
project(rplidar_hardware)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 源码目录结构
set(HOME_TREE "${CMAKE_CURRENT_SOURCE_DIR}/../")
set(MODULE_NAME "${CMAKE_PROJECT_NAME}")

set(RPLIDAR_SDK_PATH "${PROJECT_SOURCE_DIR}/sdk/")

# 包含目录
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

# 平台检测
if(APPLE)
  add_compile_definitions(_MACOS)
  FILE(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/macOS/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
else()
  FILE(GLOB RPLIDAR_SDK_SRC 
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
endif()

add_executable(${PROJECT_NAME}  src/main.cpp ${RPLIDAR_SDK_SRC})
