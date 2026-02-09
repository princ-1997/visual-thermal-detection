cmake_minimum_required(VERSION 3.10.2)

set(PROJECT_NAME "fusion_rtsp")
MESSAGE("current project: " ${PROJECT_NAME})

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_CROSSCOMPILING TRUE)

cmake_host_system_information(RESULT arch_value QUERY OS_PLATFORM)
message(STATUS "architecture: " ${arch_value})

if((NOT "${arch_value}" STREQUAL "armv7l") AND (NOT "${arch_value}" STREQUAL "aarch64"))
    include ($ENV{HOME}/configs/cross.cmake)
endif()

project(${PROJECT_NAME})
add_definitions(-Wall -Wpointer-arith)

if(CMAKE_SYSROOT)
    set(ENV{PKG_CONFIG_PATH}
        "${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig")
    set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CMAKE_SYSROOT}")
endif()

# ======================== 路径配置 =========================
set(THERMAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../thermal_rtsp")
if(NOT DEFINED EASYEAI_API_DIR)
    set(EASYEAI_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../easyeai-api" CACHE PATH "easyeai-api 根目录")
endif()
set(IR_SDK_LIBS_DIR "${THERMAL_DIR}/libs" CACHE PATH "IR SDK 动态库目录")

if(NOT EXISTS "${EASYEAI_API_DIR}/common/system_opt/api.cmake")
    message(FATAL_ERROR "找不到 easyeai-api，请指定 -DEASYEAI_API_DIR:\n"
        "  ./build.sh   (需先设置 export EASYEAI_API_DIR=/path/to/easyeai-api)\n"
        "  或 cmake .. -DEASYEAI_API_DIR=/path/to/easyeai-api\n"
        "  当前尝试路径: ${EASYEAI_API_DIR}")
endif()

# ======================== easyeai_api =========================
set(toolkit_root ${EASYEAI_API_DIR})
set(common_root ${toolkit_root}/common)
set(media_root  ${toolkit_root}/media)
set(algorithm_root ${toolkit_root}/algorithm)

include(${common_root}/system_opt/api.cmake)
include(${media_root}/gst_opt/api.cmake)
include(${algorithm_root}/person_detect/api.cmake)

set(api_inc
    ${SYSOPT_INCLUDE_DIRS}
    ${GSTOPT_INCLUDE_DIRS}
    ${CODEC_INCLUDE_DIRS}
    ${RTSP_INCLUDE_DIRS}
    ${PERSON_DETECT_INCLUDE_DIRS}
)
file(GLOB api_srcs ${SYSOPT_SOURCE_DIRS})
link_directories(${GSTOPT_LIBS_DIRS} ${CODEC_LIBS_DIRS} ${RTSP_LIBS_DIRS} ${PERSON_DETECT_LIBS_DIRS})
set(sysLib_list ${SYSOPT_LIBS} ${GSTOPT_LIBS} ${CODEC_LIBS} ${RTSP_LIBS} ${PERSON_DETECT_LIBS})

# ======================== IR SDK (复用 thermal_rtsp) =========================
set(ir_sdk_srcs
    ${THERMAL_DIR}/src/irsdk/cJSON.c
    ${THERMAL_DIR}/src/irsdk/config.cpp
    ${THERMAL_DIR}/src/irsdk/data.cpp
    ${THERMAL_DIR}/src/irsdk/cmd.cpp
)
link_directories(${IR_SDK_LIBS_DIR})
set(ir_sdk_libs ircmd iruart iruvc irv4l2 iri2c ircam irtemp irparse pthread usb-1.0)

# ======================== OpenCV =========================
# EASY-EAI-Toolkit 交叉编译：OpenCV 在 SYSROOT 中，不用 find_package
if(NOT CMAKE_SYSROOT AND DEFINED ENV{SYSROOT})
    set(CMAKE_SYSROOT $ENV{SYSROOT})
endif()
if(CMAKE_SYSROOT)
    set(OpenCV_INCLUDE_DIRS ${CMAKE_SYSROOT}/usr/include ${CMAKE_SYSROOT}/usr/include/opencv4)
    set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs)
    link_directories(${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu)
else()
    find_package(OpenCV REQUIRED)
endif()

# ======================== custom =========================
set(custom_inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camera
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${THERMAL_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
set(custom_libs m rga ${OpenCV_LIBS})

set(shared_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/fusion_streamer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/person_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/mipi_camera.c
)

#--------------------------
# fusion_rtsp: RTSP 视频流
#--------------------------
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${shared_srcs} ${ir_sdk_srcs} ${api_srcs})
target_include_directories(${PROJECT_NAME} PRIVATE ${custom_inc} ${api_inc})
target_link_libraries(${PROJECT_NAME} ${custom_libs} ${ir_sdk_libs} ${sysLib_list})

#--------------------------
# fusion_snap: 拍照工具
#--------------------------
add_executable(fusion_snap
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_snap.cpp
    ${shared_srcs} ${ir_sdk_srcs} ${api_srcs})
target_include_directories(fusion_snap PRIVATE ${custom_inc} ${api_inc})
target_link_libraries(fusion_snap ${custom_libs} ${ir_sdk_libs} ${sysLib_list})
