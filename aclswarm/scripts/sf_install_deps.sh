#!/bin/bash

# Install necessary dependencies on the Snapdragon Flight Eagle 8074.

if [[ `uname -r` != *"eagle8074" ]]; then
  echo "Must be run on a Snapdragon Flight board";
  exit -1;
fi

# build and install CMake 3.5.1 (default for Ubuntu 16.04)
VERSTR=$(cmake --version)
if [[ $VERSTR != *"3.5.1"* ]]; then
  wget http://www.cmake.org/files/v3.5/cmake-3.5.1.tar.gz
  tar zxf cmake-3.5.1.tar.gz && rm cmake-3.5.1.tar.gz
  cd cmake-3.5.1
  ./bootstrap --prefix=/usr && make -j4 install
  cd ..
  rm -rf cmake-3.5.1
else
  echo
  echo -e "Skipping CMake Install"
  echo
fi

# fix FindEigen3 errors from catkin_make
# see: https://answers.ros.org/question/215080/?answer=271618#post-id-271618
#cp /usr/share/cmake-2.8/Modules/FindEigen3.cmake /usr/share/cmake-3.5/Modules/

# Install Eigen 3.3.7 (for JacobiSVD support)
cd ~
git clone --branch 3.3.7 --depth 1 https://github.com/eigenteam/eigen-git-mirror eigen3
rm -rf /usr/include/eigen3
cd eigen3 && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make install
rm -rf eigen3
