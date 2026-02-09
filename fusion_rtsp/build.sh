#!/bin/sh

set -e

SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
cd $SHELL_FOLDER

PROJECT_NAME="fusion_rtsp"
warring() {
    echo "Thermal+Visible Fusion (RTSP + Snap)."
    echo " "
    echo "./build.sh              : build fusion_rtsp + fusion_snap"
    echo "./build.sh clear        : clear build files"
    echo " "
    echo "Environment (EASY-EAI-Toolkit 交叉编译):"
    echo "  SYSROOT            : 目标板根文件系统 (含 OpenCV: usr/include/opencv4)"
    echo "  EASYEAI_API_DIR    : easyeai-api 路径（未置时自动检测）"
    echo "  IR_SDK_LIBS_DIR    : IR SDK .so dir (default: ../thermal_rtsp/libs)"
    echo " "
    echo "若自动检测失败，请执行: export EASYEAI_API_DIR=/path/to/easyeai-api"
    echo " "
}

if [ "$1" = "clear" ]; then
    rm -rf build
    rm -rf Release
    exit 0
fi

if [ -n "$SYSROOT" ]; then
    export PKG_CONFIG_PATH="$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/share/pkgconfig"
    export PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
fi

CMAKE_ARGS=""
if [ -z "$EASYEAI_API_DIR" ]; then
    # 自动检测 easyeai-api 路径（EASY-EAI-Toolkit 典型结构）
    for candidate in "$SHELL_FOLDER/../../easyeai-api" \
                     "/opt/EASY-EAI-Toolkit/EASY-EAI-Toolkit-1126B/easyeai-api" \
                     "/opt/EASY-EAI-Toolkit/easyeai-api"; do
        if [ -f "$candidate/common/system_opt/api.cmake" ]; then
            export EASYEAI_API_DIR="$candidate"
            echo "Auto-detected EASYEAI_API_DIR: $EASYEAI_API_DIR"
            break
        fi
    done
fi
if [ -n "$EASYEAI_API_DIR" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DEASYEAI_API_DIR=$EASYEAI_API_DIR"
fi
if [ -n "$IR_SDK_LIBS_DIR" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DIR_SDK_LIBS_DIR=$IR_SDK_LIBS_DIR"
fi
if [ -n "$SYSROOT" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_SYSROOT=$SYSROOT"
fi

rm -rf build && mkdir build && cd build
cmake .. $CMAKE_ARGS
make -j$(nproc)

mkdir -p "../Release" && cp $PROJECT_NAME fusion_snap "../Release"
if [ -f "../stream_RV1126.conf" ]; then
    cp "../stream_RV1126.conf" "../Release"
fi
if [ -f "../fusion.conf" ]; then
    cp "../fusion.conf" "../Release"
fi
if [ -f "../fusion_alpha.conf" ]; then
    cp "../fusion_alpha.conf" "../Release"
fi
# IR SDK .so：优先用 IR_SDK_LIBS_DIR，否则用 thermal_rtsp/libs
LIBS_SRC="${IR_SDK_LIBS_DIR:-../../thermal_rtsp/libs}"
cp $LIBS_SRC/*.so "../Release" 2>/dev/null || true
chmod 777 ../Release -R

if [ -n "$SYSROOT" ]; then
    mkdir -p $SYSROOT/userdata/Solu/
    cp ../Release/*.so $SYSROOT/usr/lib/ 2>/dev/null || true
    ldconfig $SYSROOT 2>/dev/null || true
    cp ../Release/$PROJECT_NAME ../Release/fusion_snap $SYSROOT/userdata/Solu/
    cp ../Release/*.conf $SYSROOT/userdata/Solu/ 2>/dev/null || true
fi
exit 0
