#!/bin/sh

set -e

SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
cd $SHELL_FOLDER

# 项目名称（与CMakeLists.txt中保持一致）
PROJECT_NAME="thermal_rtsp"
warring() {
	echo "DESCRIPTION"
	echo "Thermal IR RTSP Streaming Server."
	echo " "
	echo "./build.sh              : build solution"
	echo "./build.sh clear        : clear all compiled files"
	echo " "
	echo "可选环境变量:"
	echo "  EASYEAI_API_DIR       : easyeai-api 根目录路径"
	echo "  IR_SDK_LIBS_DIR       : IR SDK 动态库(.so)目录(默认 ./libs/)"
	echo " "
	echo "示例:"
	echo "  EASYEAI_API_DIR=/opt/easyeai-api ./build.sh"
	echo " "
}

if [ "$1" = "clear" ]; then
	rm -rf build
	rm -rf Release
	exit 0
fi

# 交叉编译 pkg-config 配置（使 pkg-config 搜索 sysroot 中的 .pc 文件）
if [ -n "$SYSROOT" ]; then
	export PKG_CONFIG_PATH="$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/share/pkgconfig"
	export PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
fi

# 构建 cmake 参数
CMAKE_ARGS=""
if [ -n "$EASYEAI_API_DIR" ]; then
	CMAKE_ARGS="$CMAKE_ARGS -DEASYEAI_API_DIR=$EASYEAI_API_DIR"
fi
if [ -n "$IR_SDK_LIBS_DIR" ]; then
	CMAKE_ARGS="$CMAKE_ARGS -DIR_SDK_LIBS_DIR=$IR_SDK_LIBS_DIR"
fi

# build this project
rm -rf build && mkdir build && cd build
cmake .. $CMAKE_ARGS
make -j$(nproc)

# make Release files
mkdir -p "../Release" && cp $PROJECT_NAME "../Release"
# 拷贝配置文件到 Release
if [ -f "../stream_RV1126.conf" ]; then
	cp "../stream_RV1126.conf" "../Release"
fi
# 拷贝 IR SDK 动态库到 Release
cp ../libs/*.so "../Release" 2>/dev/null || true
chmod 777 ../Release -R

## copy to Board
mkdir -p $SYSROOT/userdata/Solu/
cp ../Release/*.so $SYSROOT/usr/lib/ 2>/dev/null || true
ldconfig $SYSROOT 2>/dev/null || true
cp ../Release/$PROJECT_NAME $SYSROOT/userdata/Solu/
cp ../Release/*.conf $SYSROOT/userdata/Solu/ 2>/dev/null || true
exit 0
