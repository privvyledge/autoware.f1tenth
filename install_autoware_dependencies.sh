#!/bin/bash
# Autoware documentation: https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/

##### (optional) Hold nvidia related packages so they are not overwritten
#sudo apt-mark hold  \
#  "cuda*"             \
#  "libcudnn*"         \
#  "libnvinfer*"       \
#  "libnvonnxparsers*" \
#  "libnvparsers*"     \
#  "tensorrt*"         \
#  "nvidia*"

################ Install dependencies
sudo apt-get -y update
sudo apt-get -y install git ccache build-essential git-lfs ssh  wget  cmake curl gosu \
  gnupg \
  vim \
  unzip \
  lsb-release

################ Setup Git LFS
git lfs install

# Install pre-commit using pip3
pip3 install pre-commit

# Install a specific version of clang-format using pip3
export pre_commit_clang_format_version=17.0.5  # todo: get from amd64.env
pip3 install clang-format==${pre_commit_clang_format_version}

# Install Go
sudo apt-get install -y golang

# Install PlotJuggler
sudo apt-get install -y ros-"${ROS_DISTRO}"-plotjuggler-ros

# Install gdown to download files from CMakeLists.txt
pip3 install gdown

sudo apt install -y geographiclib-tools

# Add EGM2008 geoid grid to geographiclib
sudo geographiclib-get-geoids egm2008-1


######## Install pacmod
# wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install -y apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install -y ros-"${ROS_DISTRO}"-pacmod3

######## Install Kisak Mesa (optional)
# Add the Kisak Mesa PPA
sudo add-apt-repository -y ppa:kisak/kisak-mesa
sudo apt-get update

# insall the MESA libraries
sudo apt-get install -y \
libegl-mesa0 \
libegl1-mesa-dev \
libgbm-dev \
libgbm1 \
libgl1-mesa-dev \
libgl1-mesa-dri \
libglapi-mesa \
libglx-mesa0


######## Install ROS dev tools (should be installed if building ROS from sources)
sudo apt update && sudo apt install -y \
  python3-colcon-mixin \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures


####################### Download and unpack artifacts. todo: make this optional
######### Download Artifacts
# yabloc_pose_initializer
mkdir -p ~/autoware_data/yabloc_pose_initializer/
wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz

# image_projection_based_fusion
mkdir -p ~/autoware_data/image_projection_based_fusion/
wget -P ~/autoware_data/image_projection_based_fusion/ \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_voxel_encoder_pointpainting.onnx \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_backbone_neck_head_pointpainting.onnx


# lidar_apollo_instance_segmentation
mkdir -p ~/autoware_data/lidar_apollo_instance_segmentation/
wget -P ~/autoware_data/lidar_apollo_instance_segmentation/ \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vlp-16.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/hdl-64.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vls-128.onnx


# lidar_centerpoint
mkdir -p ~/autoware_data/lidar_centerpoint/
wget -P ~/autoware_data/lidar_centerpoint/ \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx

# tensorrt_yolox
mkdir -p ~/autoware_data/tensorrt_yolox/
wget -P ~/autoware_data/tensorrt_yolox/ \
       https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.onnx \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt

# traffic_light_classifier
mkdir -p ~/autoware_data/traffic_light_classifier/
wget -P ~/autoware_data/traffic_light_classifier/ \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/lamp_labels.txt \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/lamp_labels_ped.txt


# traffic_light_fine_detector
mkdir -p ~/autoware_data/traffic_light_fine_detector/
wget -P ~/autoware_data/traffic_light_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_labels.txt

# tvm_utility
mkdir -p ~/autoware_data/tvm_utility/models/yolo_v2_tiny
wget -P ~/autoware_data/tvm_utility/ \
       https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/yolo_v2_tiny-x86_64-llvm-3.0.0-20221221.tar.gz

# lidar_centerpoint_tvm
mkdir -p ~/autoware_data/lidar_centerpoint_tvm/models/centerpoint_encoder
mkdir -p ~/autoware_data/lidar_centerpoint_tvm/models/centerpoint_backbone
wget -P ~/autoware_data/lidar_centerpoint_tvm/ \
       https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/centerpoint_encoder-x86_64-llvm-3.0.0-20221221.tar.gz \
       https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/centerpoint_backbone-x86_64-llvm-3.0.0-20221221.tar.gz

# lidar_apollo_segmentation_tvm
mkdir -p ~/autoware_data/lidar_apollo_segmentation_tvm/models/baidu_cnn
wget -P ~/autoware_data/lidar_apollo_segmentation_tvm/ \
      https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/baidu_cnn-x86_64-llvm-3.0.0-20221221.tar.gz

######### Extract Artifacts
# yabloc_pose_initializer
tar -xf ~/autoware_data/yabloc_pose_initializer/resources.tar.gz \
       -C ~/autoware_data/yabloc_pose_initializer/

# tvm_utility
tar -xf ~/autoware_data/tvm_utility/yolo_v2_tiny-x86_64-llvm-3.0.0-20221221.tar.gz \
       -C ~/autoware_data/tvm_utility/models/yolo_v2_tiny/

# lidar_centerpoint_tvm
tar -xf ~/autoware_data/lidar_centerpoint_tvm/centerpoint_encoder-x86_64-llvm-3.0.0-20221221.tar.gz \
       -C ~/autoware_data/lidar_centerpoint_tvm/models/centerpoint_encoder
tar -xf ~/autoware_data/lidar_centerpoint_tvm/centerpoint_backbone-x86_64-llvm-3.0.0-20221221.tar.gz \
       -C ~/autoware_data/lidar_centerpoint_tvm/models/centerpoint_backbone

# lidar_apollo_segmentation_tvm
tar -xf ~/autoware_data/lidar_apollo_segmentation_tvm/baidu_cnn-x86_64-llvm-3.0.0-20221221.tar.gz \
       -C ~/autoware_data/lidar_apollo_segmentation_tvm/models/baidu_cnn