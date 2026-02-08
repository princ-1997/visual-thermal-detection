cmake_minimum_required(VERSION 3.10.2)

# 项目名称
set(PROJECT_NAME "thermal_rtsp")
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

# 交叉编译时，设置 pkg-config 搜索 sysroot 中的 .pc 文件
if(CMAKE_SYSROOT)
    set(ENV{PKG_CONFIG_PATH}
        "${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig")
    set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CMAKE_SYSROOT}")
endif()

# ======================== 可配置路径 =========================
# 用法: cmake .. -DEASYEAI_API_DIR=/path/to/easyeai-api
#                 -DIR_SDK_LIBS_DIR=/path/to/libs

# easyeai-api 根目录
set(EASYEAI_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../easyeai-api"
    CACHE PATH "easyeai-api 根目录路径")

# IR SDK 动态库目录（.so 文件，已拷贝到 libs/）
set(IR_SDK_LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    CACHE PATH "IR SDK 动态库(.so)目录")

# 校验路径
if(NOT EXISTS "${EASYEAI_API_DIR}/common/system_opt/api.cmake")
    message(FATAL_ERROR
        "找不到 easyeai-api，请通过 -DEASYEAI_API_DIR 指定路径:\n"
        "  cmake .. -DEASYEAI_API_DIR=/your/path/to/easyeai-api\n"
        "  当前值: ${EASYEAI_API_DIR}")
endif()

# ======================== easyeai_api (GStreamer + RTSP) =========================
set(toolkit_root ${EASYEAI_API_DIR})
set(common_root    ${toolkit_root}/common)
set(media_root     ${toolkit_root}/media)

include (${common_root}/system_opt/api.cmake)
include (${media_root}/gst_opt/api.cmake)

## api头文件路径
set(api_inc
    ${SYSOPT_INCLUDE_DIRS}
    ${GSTOPT_INCLUDE_DIRS}
    ${CODEC_INCLUDE_DIRS}
    ${RTSP_INCLUDE_DIRS}
    )
## api源文件路径
file(GLOB api_srcs
    ${SYSOPT_SOURCE_DIRS}
    )
## api库文件路径
link_directories(
    ${GSTOPT_LIBS_DIRS}
    ${CODEC_LIBS_DIRS}
    ${RTSP_LIBS_DIRS}
)
## api所依赖的库
set(sysLib_list
    ${SYSOPT_LIBS}
    ${GSTOPT_LIBS}
    ${CODEC_LIBS}
    ${RTSP_LIBS}
    )
# ======================== easyeai_api =========================

# ======================== IR SDK (热红外) =========================
# IR SDK 公共模块源文件（已拷贝到 src/irsdk/ 目录，头文件在 include/）
set(ir_sdk_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/irsdk/cJSON.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/irsdk/config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/irsdk/data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/irsdk/cmd.cpp
)

# IR SDK 动态库搜索路径
link_directories(${IR_SDK_LIBS_DIR})

# IR SDK 链接库
set(ir_sdk_libs
    ircmd
    iruart
    iruvc
    irv4l2
    iri2c
    ircam
    irtemp
    irparse
    pthread
    usb-1.0
    )
# ======================== IR SDK =========================

# ========================== custom ============================
# -I（所有 IR SDK 头文件已拷贝到 include/ 目录下）
set(custom_inc
    ./src
    ${PROJECT_SOURCE_DIR}/include
)
# -l
set(custom_libs
    m
)
# source code
aux_source_directory(./src dir_srcs)
# ========================== custom ============================


#--------------------------
# execute file
#--------------------------
add_executable(${PROJECT_NAME} ${dir_srcs} ${ir_sdk_srcs} ${api_srcs})   #-o
target_include_directories(${PROJECT_NAME} PRIVATE ${custom_inc} ${api_inc})
target_link_libraries(${PROJECT_NAME} ${custom_libs} ${ir_sdk_libs} ${sysLib_list})
