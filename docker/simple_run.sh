#!/usr/bin/env bash

set -e

#IMAGE="privvyledge/r36.2.0-ros-humble-ml:latest"
IMAGE="privvyledge/f1tenth:humble-latest"
docker run -it --rm --runtime nvidia --network host --privileged \
        --volume /dev:/dev --volume ~/shared_dir:/shared_dir \
        --volume "${HOME}"/cyclone_dds_settings:"${BUILD_HOME}"/src/autoware.f1tenth/cyclone_dds \
        "${IMAGE}"