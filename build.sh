#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

MULTISTAGE_TARGET="zmq-user"

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

IMAGE_NAME=$(echo "${PWD##*/}" | tr _ -)
CONTAINER_NAME="$(echo "${PWD##*/}" | tr _ -)"
VOLUME_NAME="$(echo "${PWD##*/}" | tr _ -)_vol"

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")

if [[ "${OSTYPE}" != "darwin"* ]]; then
  UID="$(id -u "${USER}")"
  GID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${UID}")
  BUILD_FLAGS+=(--build-arg GID="${GID}")
fi
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${MULTISTAGE_TARGET}")

if [ "${REBUILD}" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit
