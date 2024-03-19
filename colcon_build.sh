#!/bin/bash

skipped_keys="rf2o_laser_odometry librealsense2 realsense2_camera libopencv-dev"
rosdep install --from-paths src --ignore-src -r -y --skip-keys "${skipped_keys}"
colcon build --symlink-install --event-handlers console_direct+ --base-paths src --cmake-args '  -Wno-dev' ' --no-warn-unused-cli' ' -DBUILD_ACCELERATE_GPU_WITH_GLSL=ON' ' -DCMAKE_BUILD_TYPE=Release' ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON' ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"  -DDOWNLOAD_ARTIFACTS=ON'