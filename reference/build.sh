#!/bin/sh

set -e

SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
cd $SHELL_FOLDER

# 项目名称（与CMakeLists.txt中保持一致）
PROJECT_NAME="realtimeDetection"
warring() {
	echo "DESCRIPTION"
	echo "EASYEAI-1126B Solution Project."
	echo " "
	echo "./build.sh       : build solution"
	echo "./build.sh clear : clear all compiled files(just preserve source code)"
	echo " "
}

if [ "$1" = "clear" ]; then
	rm -rf build
	rm -rf Release
	exit 0
fi

# build this project
rm -rf build && mkdir build && cd build
cmake ..
make -j24

# make Release files
mkdir -p "../Release" && cp $PROJECT_NAME "../Release"
# copy model file to Release directory
if [ -f "../person_detect.model" ]; then
	cp "../person_detect.model" "../Release"
	echo "Model file copied to Release directory"
fi
chmod 777 ../Release -R

## copy to Board
mkdir -p $SYSROOT/userdata/Solu/
cp ../Release/* $SYSROOT/userdata/Solu
exit 0
