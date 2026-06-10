#==============================================================================
# 玻璃清洗无人机视觉系统 - CMake 构建配置
# Glass Cleaning Drone Vision System
#==============================================================================

cmake_minimum_required(VERSION 3.15)

project(glass_cleaning_drone_vision
    VERSION 1.0.0
    DESCRIPTION "Optical flow based vision system for glass cleaning drone"
    LANGUAGES CXX
)

#------------------------------------------------------------------------------
# C++ 标准设置
#------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#------------------------------------------------------------------------------
# 默认构建类型
#------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

#------------------------------------------------------------------------------
# 项目信息输出
#------------------------------------------------------------------------------
message(STATUS "============================================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "============================================================")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")

#------------------------------------------------------------------------------
# 平台检测
#------------------------------------------------------------------------------
if(WIN32)
    set(IS_WINDOWS TRUE)
    set(IS_LINUX FALSE)
    add_definitions(-DPLATFORM_WINDOWS)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_USE_MATH_DEFINES)
elseif(UNIX AND NOT APPLE)
    set(IS_WINDOWS FALSE)
    set(IS_LINUX TRUE)
    add_definitions(-DPLATFORM_LINUX)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

#------------------------------------------------------------------------------
# 编译选项
#------------------------------------------------------------------------------
option(SIMULATION_MODE "Enable simulation mode for testing without hardware" ON)
option(USE_ORBBEC_SDK "Enable Orbbec SDK for Gemini camera support" ON)
option(ENABLE_DEBUG_VISUALIZER "Enable debug visualization windows" ON)
option(ENABLE_UART "Enable UART communication" ON)

message(STATUS "Options:")
message(STATUS "  SIMULATION_MODE: ${SIMULATION_MODE}")
message(STATUS "  USE_ORBBEC_SDK: ${USE_ORBBEC_SDK}")
message(STATUS "  ENABLE_DEBUG_VISUALIZER: ${ENABLE_DEBUG_VISUALIZER}")
message(STATUS "  ENABLE_UART: ${ENABLE_UART}")
message(STATUS "------------------------------------------------------------")

#------------------------------------------------------------------------------
# 添加编译定义
#------------------------------------------------------------------------------
if(SIMULATION_MODE)
    add_definitions(-DSIMULATION_MODE)
endif()

if(ENABLE_DEBUG_VISUALIZER)
    add_definitions(-DENABLE_DEBUG_VISUALIZER)
endif()

if(ENABLE_UART)
    add_definitions(-DENABLE_UART)
endif()

#------------------------------------------------------------------------------
# 查找依赖包
#------------------------------------------------------------------------------
message(STATUS "Finding dependencies...")

# OpenCV (必需)
find_package(OpenCV REQUIRED COMPONENTS
    core
    imgproc
    highgui
    videoio
    calib3d
    features2d
    video
    imgcodecs
)

if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenCV not found!")
endif()

# Eigen3 (必需)
find_package(Eigen3 CONFIG REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
else()
    message(FATAL_ERROR "Eigen3 not found!")
endif()

#------------------------------------------------------------------------------
# Orbbec SDK (可选)
#------------------------------------------------------------------------------
set(ORBBEC_SDK_FOUND FALSE)

if(USE_ORBBEC_SDK)
    # 设置SDK路径
    if(IS_WINDOWS)
        set(ORBBEC_SDK_ROOT "E:/develop/dev/OrbbecSDK/OrbbecSDK_v1.10.27/SDK" CACHE PATH "Orbbec SDK root directory")
    else()
        set(ORBBEC_SDK_ROOT "/usr/local/OrbbecSDK" CACHE PATH "Orbbec SDK root directory")
    endif()

    # 检查SDK是否存在
    if(EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
        set(ORBBEC_SDK_FOUND TRUE)
        set(ORBBEC_SDK_INCLUDE_DIR "${ORBBEC_SDK_ROOT}/include")

        if(IS_WINDOWS)
            set(ORBBEC_SDK_LIBRARY "${ORBBEC_SDK_ROOT}/lib/OrbbecSDK.lib")
            set(ORBBEC_SDK_DLL_DIR "${ORBBEC_SDK_ROOT}/lib")
        else()
            set(ORBBEC_SDK_LIBRARY "${ORBBEC_SDK_ROOT}/lib/libOrbbecSDK.so")
        endif()

        add_definitions(-DHAS_ORBBEC_SDK)
        add_definitions(-DUSE_ORBBEC_SDK)
        message(STATUS "Orbbec SDK found: ${ORBBEC_SDK_ROOT}")
    else()
        message(STATUS "Orbbec SDK not found at: ${ORBBEC_SDK_ROOT}")
        message(STATUS "  Camera features will be limited")
    endif()
endif()

message(STATUS "------------------------------------------------------------")

#------------------------------------------------------------------------------
# 包含目录
#------------------------------------------------------------------------------
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
    ${PROJECT_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Eigen3
if(TARGET Eigen3::Eigen)
    # 使用现代CMake目标
else()
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# Orbbec SDK
if(ORBBEC_SDK_FOUND)
    include_directories(${ORBBEC_SDK_INCLUDE_DIR})
endif()

#------------------------------------------------------------------------------
# 源文件列表
#------------------------------------------------------------------------------

# 相机模块
set(CAMERA_SOURCES
    src/camera/gemini_camera.cpp
    src/camera/camera_calibration.cpp
)

# 预处理模块
set(PREPROCESSING_SOURCES
    src/preprocessing/image_rectifier.cpp
    src/preprocessing/noise_filter.cpp
)

# 反射检测模块
set(REFLECTION_SOURCES
    src/reflection/reflection_detector.cpp
    src/reflection/edge_sharpness.cpp
    src/reflection/motion_symmetry.cpp
)

# 光流模块
set(OPTICAL_FLOW_SOURCES
    src/optical_flow/feature_detector.cpp
    src/optical_flow/flow_calculator.cpp
    src/optical_flow/depth_fusion.cpp
)

# 位移估计模块
set(ESTIMATION_SOURCES
    src/estimation/displacement_estimator.cpp
    src/estimation/kalman_filter.cpp
)

# IMU 处理模块
set(IMU_SOURCES
    src/imu/imu_processor.cpp
)

# 检测模块
set(DETECTION_SOURCES
    src/detection/window_frame_detector.cpp
)

# 配置加载模块
set(CONFIG_SOURCES
    src/config/system_config_loader.cpp
)

# 通信模块
set(COMMUNICATION_SOURCES
    src/communication/uart_interface.cpp
    src/communication/protocol.cpp
)

# 模拟器模块（可选）
set(SIMULATOR_SOURCES "")
if(SIMULATION_MODE)
    set(SIMULATOR_SOURCES
        src/simulator/glass_simulator.cpp
        src/simulator/reflection_simulator.cpp
    )
endif()

# 主程序
set(MAIN_SOURCES
    src/main.cpp
)

# 合并所有源文件
set(ALL_SOURCES
    ${MAIN_SOURCES}
    ${CAMERA_SOURCES}
    ${PREPROCESSING_SOURCES}
    ${REFLECTION_SOURCES}
    ${OPTICAL_FLOW_SOURCES}
    ${ESTIMATION_SOURCES}
    ${IMU_SOURCES}
    ${DETECTION_SOURCES}
    ${COMMUNICATION_SOURCES}
    ${CONFIG_SOURCES}
    ${SIMULATOR_SOURCES}
)

#------------------------------------------------------------------------------
# 创建可执行文件
#------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/RelWithDebInfo
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR}/MinSizeRel
)

#------------------------------------------------------------------------------
# 链接库
#------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
)

# Eigen3
if(TARGET Eigen3::Eigen)
    target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
endif()

# Orbbec SDK
if(ORBBEC_SDK_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${ORBBEC_SDK_LIBRARY})
endif()

# Linux特定库
if(IS_LINUX)
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif()

#------------------------------------------------------------------------------
# 编译选项
#------------------------------------------------------------------------------
if(MSVC)
    # MSVC编译选项
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W3          # 警告级别3
        /MP          # 多处理器编译
        /utf-8       # UTF-8源文件
        /wd4251      # 禁用DLL接口警告
        /wd4996      # 禁用弃用警告
        /wd4267      # 禁用size_t转换警告
        /wd4244      # 禁用类型转换警告
        /wd4389      # 禁用有符号/无符号不匹配警告
        /wd4100      # 禁用未引用参数警告
        /wd4458      # 禁用成员隐藏警告
    )

    # Debug模式
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(${PROJECT_NAME} PRIVATE _DEBUG)
    endif()

elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # GCC/Clang编译选项
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-unused-variable
    )

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${PROJECT_NAME} PRIVATE -g -O0)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -O3)
    endif()
endif()

#------------------------------------------------------------------------------
# 复制运行时依赖（Windows）
#------------------------------------------------------------------------------
if(IS_WINDOWS)
    # vcpkg DLL目录
    set(VCPKG_BIN_DIR "E:/develop/dev/vcpkg/installed/x64-windows/bin")
    set(VCPKG_DEBUG_BIN_DIR "E:/develop/dev/vcpkg/installed/x64-windows/debug/bin")

    # 复制vcpkg DLLs
    if(EXISTS "${VCPKG_BIN_DIR}")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "Copying vcpkg DLLs..."
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${VCPKG_BIN_DIR}"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying vcpkg runtime DLLs"
        )
    endif()

    # 复制Orbbec SDK DLLs
    if(ORBBEC_SDK_FOUND AND EXISTS "${ORBBEC_SDK_DLL_DIR}")
        file(GLOB ORBBEC_DLLS "${ORBBEC_SDK_DLL_DIR}/*.dll")
        foreach(DLL_FILE ${ORBBEC_DLLS})
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${DLL_FILE}"
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            )
        endforeach()
    endif()
endif()

#------------------------------------------------------------------------------
# 创建数据目录
#------------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/data"
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/logs"
    COMMENT "Creating data directories"
)

#------------------------------------------------------------------------------
# 构建摘要
#------------------------------------------------------------------------------
message(STATUS "============================================================")
message(STATUS "Build Summary:")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV: ${OpenCV_VERSION}")
message(STATUS "  Eigen3: ${Eigen3_VERSION}")
message(STATUS "  Simulation mode: ${SIMULATION_MODE}")
message(STATUS "  Orbbec SDK: ${ORBBEC_SDK_FOUND}")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}")
message(STATUS "============================================================")
