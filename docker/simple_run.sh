#!/usr/bin/env bash

set -e
#/f1tenth_ws/src/autoware.f1tenth/cyclone_dds/cyclonedds_config.xml
IMAGE="privvyledge/f1tenth:humble-latest"  # "privvyledge/r36.2.0-ros-humble-ml:latest"
MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
docker run -it --rm --runtime nvidia --network host  ${MOUNT_X} --privileged \
        --volume /dev:/dev --volume ~/shared_dir:/shared_dir \
        --volume /home/gosling1/cyclone_dds_settings:/f1tenth_ws/src/autoware.f1tenth/cyclone_dds \
        "${IMAGE}"
