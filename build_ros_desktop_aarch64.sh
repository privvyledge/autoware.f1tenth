#!/bin/bash

ROS_VERSION=humble
ROS_DISTRO=${ROS_VERSION}
ROS_PKG=ros_base
ROS_PYTHON_VERSION=3
# Core ROS2 workspace - the "underlay"
ROS_ROOT=/opt/ros/${ROS_DISTRO}
ROS_INSTALL_ROOT=/opt/ros/${ROS_DISTRO}
SUDO=sudo

# Setup Locale
sudo apt update && sudo apt -y install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		ca-certificates \
		lsb-release
sudo rm -rf /var/lib/apt/lists/*

# Setup source
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install curl -y
$SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install development packages
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev
sudo rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
python3 -m pip install -U \
		argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		pytest

# (optional) compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6 && \
    cd yaml-cpp-0.6 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    sudo cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    sudo ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6


# https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
BASE_PACKAGES="launch_xml \
      launch_yaml \
      launch_testing \
      launch_testing_ament_cmake \
      demo_nodes_cpp \
      demo_nodes_py \
      example_interfaces \
      xacro \
      diagnostic_updater"
PERCEPTION_PACKAGES="camera_calibration_parsers \
      camera_info_manager \
      cv_bridge \
      v4l2_camera \
      vision_opencv \
      vision_msgs \
      image_geometry \
      image_pipeline \
      image_transport \
      compressed_image_transport \
      compressed_depth_image_transport"
# to get packages required by f1tenth "rosdep keys --from-path src --ignore-packages-from-source"
# to get the commands run by rosdep,  "rosdep keys --from-path src --ignore-packages-from-source | xargs rosdep resolve"
F1TENTH_PACKAGES=""
sudo mkdir -p ${ROS_ROOT}/src && \
  cd ${ROS_ROOT}
sudo sh -c \
     "rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
       ${BASE_PACKAGES} ${PERCEPTION_PACKAGES} \
      > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
      cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
      vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall"

# Optionally get the latest ament_cmake to avoid CMake compatibility issues
rm -r ${ROS_ROOT}/src/ament_cmake
git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO}

## download unreleased packages. (optional for versions >= Humble)
#sudo sh -c "git clone --branch ros2 https://github.com/Kukanani/vision_msgs ${ROS_ROOT}/src/vision_msgs && \
#    git clone --branch ${ROS_DISTRO} https://github.com/ros2/demos demos && \
#    cp -r demos/demo_nodes_cpp ${ROS_ROOT}/src && \
#    cp -r demos/demo_nodes_py ${ROS_ROOT}/src && \
#    rm -r -f demos"

# install dependencies using rosdep
sudo apt-get update
cd ${ROS_ROOT}
SKIPPED_KEYS="console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers qt_gui libopencv-dev  libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"
sudo rosdep init
    sudo apt update && rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys "${SKIPPED_KEYS}" && \
    sudo rm -rf /var/lib/apt/lists/*

# build/compile ROS2
sudo mkdir -p ${ROS_INSTALL_ROOT}
# sudo required to write build logs
sudo colcon build --merge-install --install-base ${ROS_INSTALL_ROOT} --cmake-args ' -DCMAKE_BUILD_TYPE=Release'
# Repeat the build process as not all packages build the first time
sudo colcon build --merge-install --install-base ${ROS_INSTALL_ROOT} --cmake-args ' -DCMAKE_BUILD_TYPE=Release'

# Using " expands environment variable immediately
echo "source "${ROS_INSTALL_ROOT}"/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc

# Optionally remove build files to free up space
sudo rm -rf ${ROS_ROOT}/src ${ROS_ROOT}/logs ${ROS_ROOT}/build ${ROS_ROOT}/*.rosinstall

# cleanup apt
sudo rm -rf /var/lib/apt/lists/*
sudo apt-get clean