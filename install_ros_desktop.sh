#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
SUDO=""

# Install $SUDO if not installed
apt update && apt install -y $SUDO
$SUDO apt update && $SUDO apt upgrade -y # could skip this step if it breaks the system packages

# Purge or hold opencv so it is not overwritten by ros or cause installation issues
# Todo: has issues due to Opencv. Try: https://github.com/dusty-nv/jetson-containers/issues/158#issuecomment-1292770467
# $SUDO apt-mark hold *opencv*  # Doesn't work. do this before setting up ROS packages so ros packages with the name "opencv" are not held or use a better regex.
$SUDO apt purge -y *opencv*  # todo: find a fix for this

# Setup locale
$SUDO apt update && $SUDO apt install -y locales
$SUDO locale-gen en_US en_US.UTF-8
$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
$SUDO apt install -y software-properties-common
$SUDO add-apt-repository -y universe
$SUDO apt update && $SUDO apt install curl -y
$SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS
# Todo: set non interactive environment to avoid having to select region/location
$SUDO apt update && $SUDO apt upgrade -y # could skip this step if it breaks the system packages
$SUDO apt update && DEBIAN_FRONTEND="noninteractive" $SUDO apt install -y ros-humble-desktop ros-dev-tools
$SUDO apt install -y ros-humble-rmw-cyclonedds-cpp
$SUDO rm -rf /var/lib/apt/lists/*

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$SUDO apt upgrade -y
$SUDO rosdep init
rosdep update
#rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers libopencv-dev"

# Environment setup (~/.bashrc, /etc/bash.bashrc)
# todo: ROS_DISTRO, ROS_ROOT, ROS_PACKAGE environment variables should be set
#{
#  echo "source /opt/ros/humble/setup.bash"
#  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
#  echo "export RCUTILS_COLORIZED_OUTPUT=1"
#  echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"'
#  echo source `pwd`/ros2_ws/install/local_setup.bash
#  echo "export CYCLONEDDS_URI=file://${HOME}/cyclone_dds_settings/cyclonedds_config.xml"
#  echo "export CCACHE_DIR=/ccache"
#  echo "export CC='/usr/lib/ccache/gcc'"
#  echo "export CXX='/usr/lib/ccache/g++'"
#} >> ~/.bashrc

#echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
#echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
#echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
#echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"' >> ~/.bashrc
#echo source `pwd`/ros2_ws/install/local_setup.bash >> ~/.bashrc
#echo "export CYCLONEDDS_URI=file:///home/gosling1/cyclone_dds_settings/cyclonedds_config.xml" >> ~/.bashrc
#echo "export CCACHE_DIR=/ccache" >> ~/.bashrc
#echo "export CC='/usr/lib/ccache/gcc'" >> ~/.bashrc
#echo "export CXX='/usr/lib/ccache/g++'" >> ~/.bashrc

