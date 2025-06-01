#!/bin/bash

IMAGE_NAME="bv_container" # Leaving this as dreamer_docker for now because I don't want to build a redundant docker on the raspi
CONTAINER_NAME="bv_container"
HOST_DIR="/home/bvorinnano/bv_ws"
CONTAINER_SRC_DIR="/bv_ws"

docker rm $CONTAINER_NAME

docker run -it \
    --name "$CONTAINER_NAME" \
    --network host \
    --privileged \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    --env DISPLAY="$DISPLAY" \
    -v "$HOST_DIR":"$CONTAINER_SRC_DIR" \
    "$IMAGE_NAME" \
    bash
