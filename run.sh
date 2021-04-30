#!/usr/bin/env bash

NAME=$(echo "${PWD##*/}" | tr _ -)
MULTISTAGE_TARGET="zmq-user"
#MULTISTAGE_TARGET="ros-user"

USE_NVIDIA_TOOLKIT=false
[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="host" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  --env XAUTHORITY="$XAUTH" \
  --env DISPLAY="${DISPLAY}" \
  "${NAME}:${MULTISTAGE_TARGET}"
