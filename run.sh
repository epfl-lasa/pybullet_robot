#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

IMAGE_NAME=$(echo "${PWD##*/}" | tr _ -)
MULTISTAGE_TARGET="zmq-user"

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="host" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume=$XAUTHORITY:$XAUTHORITY \
  --env XAUTHORITY=$XAUTHORITY \
  --env DISPLAY=$DISPLAY \
  ${IMAGE_NAME}:${MULTISTAGE_TARGET}
